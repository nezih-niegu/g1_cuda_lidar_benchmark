# G1 MuJoCo / ROS 2 Humble CUDA LiDAR benchmark

**Research prototype, not a verified turn-key build.** This package is designed to integrate with DFKI's [mujoco_ros2_control](https://github.com/dfki-ric/mujoco_ros2_control) `main` branch (its documented Humble branch), launching the **Unitree G1** humanoid example. The upstream simulator, model assets and LiDAR remain upstream rather than being duplicated. Verify the checked-out upstream commit's G1 demo/LiDAR support on Humble: features and documentation may differ across upstream branches. `unitree_g1.launch.py` and `/head/points` are *documented defaults for the newer G1 example*, not guaranteed for every `main` revision. Pin the exact upstream commit in a published experiment.

This benchmark is a **LiDAR filtering + bounded voxel-map visualization** experiment, **not full SLAM** (no scan matching, loop closure, or pose estimation). A fixed humanoid is sufficient for a local map. For a *moving* humanoid, configure an independently provided TF world/map transform: otherwise clouds measured in the moving sensor frame cannot be accumulated as a world map. If upstream cannot supply that TF, keep `map_frame:=''` and report sensor-frame-only mapping.

## Architecture

```text
DFKI MuJoCo / Unitree G1 head LiDAR (upstream sensor)  [MuJoCo world]
       | /head/points  sensor_msgs/PointCloud2, timestamp + frame_id
       v
Node A: g1_lidar_dds_publisher [sensor_relay.py]
       | /benchmark/raw_points ; DDS BEST_EFFORT depth=5
       v
Node B: lidar_mapper [mapper.cpp] mode=cpu | naive | optimized
       | -- xyz unpack -> identical radial/finite/voxel filter
       | -- optional TF transform to fixed frame -> bounded voxel map
       | -- /benchmark/filtered_points and /benchmark/map_points
       v
RViz2 PointCloud2 display and per-frame CSV; NVIDIA profilers on Node B
```

All three variants use the same LiDAR producer, ROS topic type, QoS, radius thresholds, voxel size, first-observed-point policy, accumulator and visualizer. Naive GPU copies voxel keys back to CPU and deduplicates there; optimized GPU reuses allocated device buffers, filters/sorts/uniques on-device, returning only chosen indices. The CPU implementation is the reference. GPU backends currently optimize only *per-scan filtering*; common map accumulation and PointCloud2 serialization remain on the CPU and are measured separately.

**Important experiment validity:** Optimized device sort is not guaranteed to outperform CPU for small scans. CUDA and MuJoCo visual rendering may contend for the GPU. Report both the filter-only and full-callback statistics rather than claiming speedups in advance. NVTX markers exist on the node's main stages; CUDA API and kernels appear in Nsight Systems. `-lineinfo` enables Nsight Compute source correlation.

## Requirements

- Ubuntu 22.04, ROS 2 Humble, `colcon`, `rosdep`, RViz2, a supported NVIDIA GPU/driver and a compatible installed CUDA Toolkit (`nvcc` and Nsight tools).
- DFKI's upstream MuJoCo/ros2_control stack, Unitree G1 assets, and its example package built for **Humble**. Upstream installs have additional dependencies and potentially MuJoCo licensing/build requirements. See the pinned upstream README before installation.
- `libnvtx3-dev` or NVTX3 headers; CUDA Toolkit containing `nvcc` and Thrust. Install ROS dependencies using `rosdep` before build. GPU-only CUDA installation is not required if `-DBUILD_CUDA=OFF` is selected.

### 1. Prepare the Humble workspace

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/g1_ws/src
cd ~/g1_ws/src
git clone -b main https://github.com/dfki-ric/mujoco_ros2_control.git
# Copy the unzipped directory g1_cuda_lidar_benchmark here:
# cp -r /path/to/g1_cuda_lidar_benchmark ./
cd ~/g1_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
# The DFKI upstream project may need extra system/model dependencies:
# follow that revision's installation notes first.
colcon build --symlink-install --cmake-args -DBUILD_CUDA=ON
source install/setup.bash
```

If ROS 2 Humble's Ubuntu 22.04 package does not provide usable NVTX3 headers, install the CUDA Toolkit's matching NVTX3 development headers. Re-run `colcon build`. Avoid mixing different MuJoCo control repositories or Jazzy binaries in the same Humble workspace.

### 2. Launch the *upstream* humanoid simulator

```bash
# Terminal A
source ~/g1_ws/install/setup.bash
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py
```

**Do this before starting the benchmark and inspect the actual graph:**

```bash
ros2 topic list -t | grep -E 'points|scan|clock|tf'
ros2 topic info /head/points -v
ros2 topic hz /head/points
```

Confirm the upstream G1 LiDAR publishes `sensor_msgs/msg/PointCloud2`. If your Humble `main` checkout lacks the G1+LiDAR example (or uses different topic naming), you must port/enable its G1 LiDAR scene/configuration from a compatible upstream revision; do **not** pretend synthetic generated clouds are MuJoCo LiDAR. Alternatively run an upstream branch/revision whose G1 example and dependencies are known to build for Humble. Pass the observed LiDAR topic to `source_topic:=...`.

### 3. Run a mode

```bash
# Terminal B, always source the same workspace
source ~/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # requires ros-humble-rmw-cyclonedds-cpp
ros2 launch g1_cuda_lidar_benchmark benchmark.launch.py \
  source_topic:=/head/points backend:=cpu csv_path:=/tmp/g1_cpu.csv
```

Repeat (one mapper process at a time) with `backend:=naive csv_path:=/tmp/g1_naive.csv verify:=true` and `backend:=optimized csv_path:=/tmp/g1_optimized.csv verify:=true`. Record fixed-rate, identical input with `ros2 bag` for the real benchmark rather than relying on live randomized worlds:

```bash
ros2 bag record -o /tmp/g1_reference /benchmark/raw_points /clock /tf /tf_static
```

When replaying a bag, run only the mapper with `relay:=false`. Replay the same bag for each backend. Do **not** run the MuJoCo simulator concurrently with bag playback unless you intentionally test GPU contention. Ensure `/clock` and TF playback if using `map_frame`, and check the `/benchmark/raw_points` publisher count is exactly one.

```bash
# Terminal B: backend mapper only (repeat for each backend)
ros2 launch g1_cuda_lidar_benchmark benchmark.launch.py relay:=false \
  backend:=cpu csv_path:=/tmp/g1_cpu.csv
# Terminal C: after mapper has subscribed
ros2 bag play /tmp/g1_reference
```

**Time stamps:** Our relay preserves incoming simulation stamps. `stamp_ns` and `recv_ros_ns` are logged, but use wall-clock monotonic per-callback timing for latency stats; simulation-time stamping, bag clock delivery and cross-process wall-time require careful interpretation. To get true publisher-to-subscriber DDS delay, add synchronized steady-clock timestamps via a custom message on one machine or clock synchronization across hosts, and account for serialization time. Do not subtract simulation timestamps from steady-clock times.

### 4. Render the map

```bash
rviz2
```

Add two PointCloud2 displays: `/benchmark/filtered_points` and `/benchmark/map_points`. Set RViz **Fixed Frame** equal to the actual incoming cloud's `header.frame_id` in fixed-pose mode, or to your TF-backed `map_frame`. Configure display QoS Reliability to Best Effort. If using world mapping, pass `map_frame:=world` or another available TF target and verify transforms exist at the original LiDAR timestamp.

## Profiling pipeline

Run an **unprofiled** warmup/reference pass and close RViz for throughput comparisons; enable RViz only when testing visualization cost. When possible keep simulation and inference on separate GPUs or stop simulator and replay a rosbag to avoid uncontrolled graphics interference.

```bash
mkdir -p ~/g1_profiles
# Profile executable directly; ROS arguments after --ros-args:
nsys profile --trace=cuda,nvtx,osrt --sample=none --delay=10 --duration=30 \
  -o ~/g1_profiles/naive \
  ros2 run g1_cuda_lidar_benchmark lidar_mapper --ros-args \
    -p backend:=naive -p csv_path:=/tmp/g1_naive_nsys.csv
# In another shell: replay the bag once the subscription is ready.
nsys stats --report cuda_api_sum --report cuda_gpu_kern_sum ~/g1_profiles/naive.nsys-rep
```

Repeat with optimized; Nsight Systems reveals CPU/GPU overlap, memory copies and GPU idle regions. Select an identified expensive kernel with **Nsight Compute** only during isolated/safe profiling runs (Thrust may emit multiple mangled kernel names; list them first):

```bash
ncu --target-processes all --set basic --kernel-name-base demangled \
  --launch-count 10 -o ~/g1_profiles/optimized_ncu \
  ros2 run g1_cuda_lidar_benchmark lidar_mapper --ros-args -p backend:=optimized
```

NVIDIA profilers can modify execution speed significantly: use their timelines to guide optimization, not as your final latency measurements. Keep `verify_against_cpu:=true` on correctness runs, `false` on final timing runs (the reference verification itself is expensive). Hardware traces of **DDS middleware** can be collected separately with `ros2_tracing`; the CSV is for callback/filter/publish latency, not a network-layer DDS trace.

### 5. Summarize actual results

```bash
ros2 run g1_cuda_lidar_benchmark summarize.py \
  /tmp/g1_cpu.csv /tmp/g1_naive.csv /tmp/g1_optimized.csv \
  --warmup 100 --deadline-ms 100 --out /tmp/g1_comparison.csv
```

Target 10 Hz scans and a 100 ms callback deadline initially, then sweep points per cloud, scan frequency (10/20/30 Hz as supported by upstream), voxel sizes (0.05/0.10/0.20 m), and bounded map capacity. Use identical dataset, CPU/GPU power configuration, DDS RMW, QoS and map initialization for all modes. Run at least five independent repetitions and provide confidence intervals; avoid treating adjacent autocorrelated frames as independent repetitions. Collect DDS drop counts separately with a frame-sequence custom message if losses are part of your evaluation: `PointCloud2` itself does not carry a sequence number.

### Reproducibility and limitations

- The map is deliberately a bounded **first-point voxel accumulator**, not scan registration or occupancy mapping. Once `max_map_voxels` is reached, it retains existing voxels and ceases adding new ones. `map_frame` should only be configured if real TF is available.
- CPU and GPU use double-precision coordinate/voxel computations for parity. FP implementations can disagree *at exact voxel boundaries*; report mismatches and investigate rather than silently relaxing the definition. The chosen-first-point rule is deterministic for identical input order. Cross-device bitwise equivalence is not guaranteed for arbitrary floating point data.
- The optimized backend uses an on-GPU stable sort and unique, not a claim of globally optimal CUDA design; alternate implementations (radix sort, CUB, stream pipelines, pinned buffers or CUDA Graphs) belong in subsequent measured optimization experiments.
- This code has undergone static checks in the file-generation environment but has **not** been compiled in ROS 2 Humble or executed against MuJoCo/CUDA here. Build/run and pin your actual upstream commit before publishing results.

## Experiment outputs

`/benchmark/raw_points`, `/benchmark/filtered_points`, `/benchmark/map_points`, a CSV per mode, optional rosbag input/TF, Nsight Systems and Compute reports, and `/tmp/g1_comparison.csv` produced by `summarize.py`.

## Included development aids

- `launch/smoke.launch.py` + `scripts/synthetic_lidar.py`: deterministic **non-MuJoCo** DDS integration smoke test. Run `ros2 launch g1_cuda_lidar_benchmark smoke.launch.py backend:=cpu` and inspect `/benchmark/map_points` or `/tmp/g1_smoke.csv`.
- `scripts/run_three_modes.sh`: automates five separate CPU/naive/optimized rosbag replay passes by default. Source the built Humble workspace first, stop the simulator and relay, then run `RUNS=5 bash src/g1_cuda_lidar_benchmark/scripts/run_three_modes.sh /tmp/g1_reference /tmp/g1_experiment`. Beware: ROS bag may need more startup time; `SLEEP_READY=5` adjusts this. Set `PLAY_CLOCK=1` only for bags without a recorded `/clock` topic, when a generated playback clock is needed. A single bag replay is not guaranteed to deliver exactly equal callback counts under BEST_EFFORT, so compare `frames` and check delivery loss before drawing conclusions.
- `scripts/collect_system_info.sh`: writes reproducibility metadata, including NVIDIA driver/toolkit and upstream checkout revision.
- `scripts/summary_across_runs.py`: computes run-level means and 95% t-confidence intervals (normal approximation for 5+ runs); do not mistake per-frame correlations for independent replicates.
- `tests/`: standard-library offline tests for CSV reports and source-level scaffolding, runnable without a ROS installation with `python3 -m unittest discover -s tests -v`.

### Optional DDS configuration

Use `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. For a single-host fixed-domain test, an example CycloneDDS configuration is in `config/cyclonedds.xml` (set `CYCLONEDDS_URI=file://.../config/cyclonedds.xml`). Verify the exact config format supported by your installed CycloneDDS version. Use `ros2 topic info /benchmark/raw_points -v` to verify BEST_EFFORT compatibility and monitor publisher/subscriber counts. Latency in these CSVs includes process-local decoding/filter/mapping/publishing, **not** upstream sensor-to-DDS transit latency.

### Recommended run protocol

1. Start the upstream G1 simulation, confirm the actual point topic, start only the relay, and record a representative rosbag. Stop both afterward.
2. Inspect one capture with `ros2 bag info /tmp/g1_reference` and verify it contains `/benchmark/raw_points`. Freeze parameter values, upstream commit, model assets, scene, DDS implementation, ROS_DOMAIN_ID and GPU power settings.
3. Smoke-test CPU and both GPU modes using `smoke.launch.py`; enable GPU reference checking using launch `verify:=true` or CLI `-p verify_against_cpu:=true` in correctness-only trials.
4. Perform the repeated *unprofiled* three-backend runs using the exact same bag. Keep RViz closed during compute comparisons. For mapping visual checks, open RViz2 separately and inspect both map and filtered topics.
5. Profile naive and optimized CUDA separately (`nsys` first, then `ncu`) and keep those samples out of final time statistics. Record the timeline and kernel profile files alongside the experiment CSVs.
6. Analyze each replay separately, then aggregate run-level results. Confirm exact retained-index equality on GPU correctness runs; review dropped-frame counts and GPU profiler overhead before drawing conclusions.

### Extended experiments

Try a controlled scan-density sweep by changing the *upstream* G1 LiDAR configuration (or replaying independently captured density variants), not by silently replacing the MuJoCo data with synthetic scans. For end-to-end DDS transport measurement, a future custom stamped header must propagate a monotonic time reference across both processes on the same machine (or explicitly synchronized clocks on different machines); the original LiDAR simulation stamp is not that reference.

### Starter RViz view and profiler helper

Run `ros2 launch g1_cuda_lidar_benchmark visualize.launch.py` for a saved RViz layout (its initial Fixed Frame is `lidar_smoke_frame` for the synthetic smoke test; **change it** to the real MuJoCo LiDAR frame or TF-backed target for upstream data). `bash src/g1_cuda_lidar_benchmark/scripts/profile_mapper.sh optimized ~/g1_profiles/optimized nsys` is a convenience wrapper for Nsight Systems; replay the recorded bag in another terminal while the mapper is recording. Nsight Compute is available as a third argument `ncu`, but may need additional GPU profiling permissions on some systems.
