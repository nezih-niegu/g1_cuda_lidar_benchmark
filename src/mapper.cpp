#include "g1_cuda_lidar_benchmark/filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#if __has_include(<nvtx3/nvtx3.hpp>)
#include <nvtx3/nvtx3.hpp>
#else
namespace nvtx3 { class scoped_range { public: explicit scoped_range(const char*) {} }; }
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
using g1bench::Point;
using g1bench::FilterParams;
using Cloud=sensor_msgs::msg::PointCloud2;
using Clock=std::chrono::steady_clock;
namespace {
uint64_t key(Point p, FilterParams f) {
  auto x=static_cast<long long>(std::floor(double(p.x)/f.voxel_size));
  auto y=static_cast<long long>(std::floor(double(p.y)/f.voxel_size));
  auto z=static_cast<long long>(std::floor(double(p.z)/f.voxel_size));
  if(x < -1048576 || x > 1048575 || y < -1048576 || y > 1048575 || z < -1048576 || z > 1048575) return UINT64_MAX;
  return (uint64_t(x+1048576)<<42) | (uint64_t(y+1048576)<<21) | uint64_t(z+1048576);
}
std::vector<uint32_t> cpu_filter(const std::vector<Point>& pts,FilterParams f) {
  nvtx3::scoped_range range{"cpu_filter_voxel"};
  std::unordered_set<uint64_t> seen;
  std::vector<uint32_t> out;
  for(uint32_t i=0;i<pts.size();++i){
    auto p=pts[i];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) continue;
    double r2=double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z;
    if(r2<double(f.min_range)*f.min_range||r2>double(f.max_range)*f.max_range) continue;
    auto k=key(p,f);
    if(k!=UINT64_MAX&&seen.insert(k).second) out.push_back(i);
  }
  return out;
}
// Explicit wire-level PointCloud2 XYZ decode: supports padding and arbitrary x/y/z float32 offsets.
std::vector<Point> unpack(const Cloud& cloud) {
  int ox=-1,oy=-1,oz=-1;
  for(auto& f:cloud.fields){
    if(f.datatype!=sensor_msgs::msg::PointField::FLOAT32) continue;
    if(f.name=="x") ox=int(f.offset);
    if(f.name=="y") oy=int(f.offset);
    if(f.name=="z") oz=int(f.offset);
  }
  if(ox<0||oy<0||oz<0||cloud.point_step==0 ||
     ox+4>int(cloud.point_step)||oy+4>int(cloud.point_step)||oz+4>int(cloud.point_step) || cloud.is_bigendian)
    throw std::runtime_error("Expected little-endian PointCloud2 with float32 x/y/z");
  if(cloud.row_step<size_t(cloud.width)*cloud.point_step ||
     cloud.data.size()<size_t(cloud.row_step)*cloud.height)
    throw std::runtime_error("Malformed PointCloud2 dimensions");
  std::vector<Point> pts; pts.reserve(size_t(cloud.width)*cloud.height);
  for(uint32_t row=0;row<cloud.height;++row){
    for(uint32_t col=0;col<cloud.width;++col){
      const auto* b=cloud.data.data()+size_t(row)*cloud.row_step+size_t(col)*cloud.point_step;
      Point p; std::memcpy(&p.x,b+ox,4);std::memcpy(&p.y,b+oy,4);std::memcpy(&p.z,b+oz,4);
      pts.push_back(p);
    }
  }
  return pts;
}
Cloud pack(const std::vector<Point>& pts, const std_msgs::msg::Header& header){
  Cloud cloud;sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(pts.size());
  cloud.header=header; cloud.is_dense=false;
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
  for(auto p:pts){ *x=p.x;*y=p.y;*z=p.z;++x;++y;++z; }
  return cloud;
}
double millis(Clock::time_point t,Clock::time_point u){return std::chrono::duration<double,std::milli>(u-t).count();}
}
class Mapper:public rclcpp::Node {
 public:
 Mapper():Node("lidar_mapper") {
  backend_=declare_parameter<std::string>("backend","cpu");
  input_=declare_parameter<std::string>("input_topic","/benchmark/raw_points");
  map_frame_=declare_parameter<std::string>("map_frame","");
  FilterParams f{float(declare_parameter<double>("min_range",0.2)),float(declare_parameter<double>("max_range",20.0)),
                 float(declare_parameter<double>("voxel_size",0.10))}; f_=f;
  max_map_=size_t(declare_parameter<int>("max_map_voxels",300000));
  map_voxel_=float(declare_parameter<double>("map_voxel_size",0.1));
  csv_path_=declare_parameter<std::string>("csv_path","/tmp/g1_bench_cpu.csv");
  verify_=declare_parameter<bool>("verify_against_cpu",false);
  if(f_.voxel_size<=0||map_voxel_<=0||f_.max_range<=f_.min_range||max_map_==0)
     throw std::runtime_error("Invalid filter or map parameters");
#ifndef ENABLE_CUDA
  if(backend_!="cpu") throw std::runtime_error("Built without CUDA; rebuild with -DBUILD_CUDA=ON");
#endif
  if(backend_!="cpu"&&backend_!="naive"&&backend_!="optimized") throw std::runtime_error("backend must be cpu, naive or optimized");
  if(!map_frame_.empty()) {
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  csv_.open(csv_path_,std::ios::out|std::ios::trunc);
  if(!csv_) throw std::runtime_error("Cannot open CSV: "+csv_path_);
  csv_<<"seq,backend,n_input,n_filtered,map_size,decode_ms,filter_ms,map_ms,publish_ms,callback_ms,reference_equal,stamp_ns,recv_ros_ns,done_ros_ns\n";
  auto qos=rclcpp::SensorDataQoS().keep_last(5);
  sub_=create_subscription<Cloud>(input_,qos,[this](Cloud::ConstSharedPtr cloud){callback(cloud);});
  pub_=create_publisher<Cloud>("/benchmark/map_points",rclcpp::QoS(1).best_effort());
  filtered_pub_=create_publisher<Cloud>("/benchmark/filtered_points",rclcpp::SensorDataQoS());
  RCLCPP_INFO(get_logger(),"backend=%s input=%s map_frame=%s",backend_.c_str(),input_.c_str(),map_frame_.empty()?"sensor-local":map_frame_.c_str());
 }
 private:
 void callback(const Cloud::ConstSharedPtr& cloud) {
  nvtx3::scoped_range all{"ROS_callback"};
  const auto start=Clock::now();const auto recv_ros=now();
  try {
    std::vector<Point> points;
    { nvtx3::scoped_range region{"pointcloud_decode"}; points=unpack(*cloud); }
    auto t_decode=Clock::now();
    std::vector<uint32_t> indices;
    { nvtx3::scoped_range region{"filter_backend"};
      if(backend_=="cpu") indices=cpu_filter(points,f_);
#ifdef ENABLE_CUDA
      else if(backend_=="naive") indices=g1bench::cuda_naive_indices(points,f_);
      else indices=g1bench::cuda_optimized_indices(points,f_);
#endif
    }
    auto t_filter=Clock::now();
    bool equals=true;
    if(verify_&&backend_!="cpu") {
      auto ref=cpu_filter(points,f_);
      equals=(ref==indices); // exact selected-index equivalence
      if(!equals) RCLCPP_ERROR(get_logger(),"Mismatch on frame %lu",(unsigned long)seq_);
    }
    std::vector<Point> filtered;filtered.reserve(indices.size());
    for(auto i:indices) filtered.push_back(points.at(i));
    auto filtered_msg=pack(filtered,cloud->header);
    std_msgs::msg::Header map_header=cloud->header;
    // For a moving humanoid, transform retained points before accumulating a WORLD map.
    // Filtering itself is in the local sensor frame, identically across backends.
    if(!map_frame_.empty()&&cloud->header.frame_id!=map_frame_) {
      auto tf=tf_buffer_->lookupTransform(map_frame_,cloud->header.frame_id,cloud->header.stamp,
                                           tf2::durationFromSec(0.05));
      const auto& q=tf.transform.rotation;const auto& t=tf.transform.translation;
      double xx=q.x,yy=q.y,zz=q.z,ww=q.w;
      for(auto& p:filtered){
        double x=p.x,y=p.y,z=p.z;
        // Cross-product quaternion rotation equivalent to R(q)*p.
        double tx=2*(yy*z-zz*y),ty=2*(zz*x-xx*z),tz=2*(xx*y-yy*x);
        p.x=float(x+ww*tx+(yy*tz-zz*ty)+t.x);
        p.y=float(y+ww*ty+(zz*tx-xx*tz)+t.y);
        p.z=float(z+ww*tz+(xx*ty-yy*tx)+t.z);
      }
      map_header.frame_id=map_frame_;
    } else if(!map_frame_.empty()) map_header.frame_id=map_frame_;
    {
      nvtx3::scoped_range region{"integrate_voxel_map"};
      FilterParams mf{0,10000,map_voxel_};
      // A bounded first-observation voxel map; stop accumulating at capacity.
      for(auto p:filtered) {
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) continue;
        auto k=key(p,mf);
        if(k!=UINT64_MAX&&map_.size()<max_map_)map_.emplace(k,p);
      }
    }
    auto t_map=Clock::now();
    std::vector<Point> map_points;map_points.reserve(map_.size());
    // Stable visualization ordering independent of hash-table implementation.
    std::vector<uint64_t> keys;keys.reserve(map_.size());
    for(auto& kv:map_)keys.push_back(kv.first);
    std::sort(keys.begin(),keys.end());
    for(auto k:keys) map_points.push_back(map_.at(k));
    {nvtx3::scoped_range region{"serialize_publish"};
      filtered_pub_->publish(std::move(filtered_msg));
      pub_->publish(pack(map_points,map_header));
    }
    auto end=Clock::now();auto done_ros=now();
    int64_t stamp_ns=rclcpp::Time(cloud->header.stamp).nanoseconds();
    csv_<<seq_++<<','<<backend_<<','<<points.size()<<','<<indices.size()<<','<<map_.size()<<','
      <<millis(start,t_decode)<<','<<millis(t_decode,t_filter)<<','<<millis(t_filter,t_map)<<','
      <<millis(t_map,end)<<','<<millis(start,end)<<','<<(equals?1:0)<<','<<stamp_ns<<','
      <<recv_ros.nanoseconds()<<','<<done_ros.nanoseconds()<<'\n';
    csv_.flush();
  } catch(const std::exception& e) {
    ++failed_;RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Frame processing failed (%lu): %s",failed_,e.what());
  }
 }
 std::string backend_,input_,map_frame_,csv_path_;
 FilterParams f_{};float map_voxel_{};size_t max_map_{};bool verify_{};
 uint64_t seq_=0,failed_=0;std::ofstream csv_;
 std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
 std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
 std::unordered_map<uint64_t,Point> map_;
 rclcpp::Subscription<Cloud>::SharedPtr sub_;
 rclcpp::Publisher<Cloud>::SharedPtr pub_,filtered_pub_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Mapper>());}catch(const std::exception& e){fprintf(stderr,"Fatal: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
