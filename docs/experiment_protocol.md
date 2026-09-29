# Controlled experiment protocol

**Primary endpoint**: ROS 2 mapper callback p99 (milliseconds); **secondary**: filtering stage p50/p99; **correctness**: identical retained indices compared with CPU baseline on the same PointCloud2 messages.

- Freeze MuJoCo G1 upstream checkout (record `git rev-parse HEAD`), scene, camera/LiDAR settings, point density, recording duration and sensor frames.
- Record a rosbag with only the simulation and LiDAR relay active; capture the exact `/benchmark/raw_points`, `/clock` and any needed `/tf` and `/tf_static` topics. Do not conflate the sensor's original upstream publisher with the relay: there are logically two DDS hops for live data but only one producer during bag replay.
- Replay that same bag for all backends, with independent mapper processes and fresh maps. Confirm there is exactly one publisher on `/benchmark/raw_points`.
- Use BEST_EFFORT depth=5 throughout relay and mapper. Record DDS implementation, QoS, sample counts and losses; bag replay delivery rates can differ under load. Separate middleware timing with ros2_tracing as needed.
- Run 5+ independent repetitions per backend; rotate mode order in later experiments to reduce temperature/drift bias (the provided convenience runner uses fixed order).
- Reserve correctness runs (verification ON) separately from timing runs (verification OFF); a CPU reference computation inside GPU timing callbacks would invalidate comparisons.
- Compare both end-to-end callback distributions and stage-level timing. GPU timings include transfer and device allocation (naive), or buffer reuse and sort/unique (optimized), while serialization/map integration are common.
- Close RViz for compute-only results; assess RViz update rate separately. For moving robots, publish an authoritative world-to-sensor TF before claiming world mapping.
- Profile with Nsight Systems to identify bottlenecks. Profile specific kernels with Nsight Compute and take unprofiled final measurements. Report regressions and counterexamples; do not presume optimized CUDA wins every workload.

**Validity limitations:** This is a filtering and first-observation voxel accumulation experiment, not localization or full SLAM. A real SLAM experiment needs pose estimation, data association, map error metrics and ground-truth comparison. A CUDA-optimized per-scan filter may be bottlenecked elsewhere in the overall ROS 2 pipeline.
