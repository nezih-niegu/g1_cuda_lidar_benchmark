# Distribution compatibility: Humble and Jazzy

This package uses the same ROS interfaces, C++17 filtering pipeline, CUDA implementations, PointCloud2 serialization, DDS BEST_EFFORT topic and benchmark scripts on both supported distributions.

| Distribution | Supported host (binary install) | DFKI repository branch | Our code |
|---|---|---|---|
| Humble | Ubuntu 22.04 (Jammy) | `main` | Shared |
| Jazzy | Ubuntu 24.04 (Noble) | `jazzy` | Shared |

Upstream branch reference: https://github.com/dfki-ric/mujoco_ros2_control/tree/jazzy

## Reproducible startup

In a fresh shell, use `source /opt/ros/humble/setup.bash` OR `source /opt/ros/jazzy/setup.bash` followed by `source ~/g1_ws/install/setup.bash`. Never mix distributions or reuse one distribution's colcon `build/`, `install/`, `log/` directories for the other. Use separate Ubuntu installations, VMs, or containers if comparing both.

`bash scripts/setup_workspace.sh` auto-detects the distribution from Ubuntu; `bash scripts/setup_workspace.sh jazzy` or `humble` selects explicitly, provided the relevant ROS distro is installed. On an existing upstream checkout, the bootstrap script refuses an unexpected branch rather than changing it automatically.

Install your distribution's RMW: `sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp`. If selected, use it consistently for the producer, mapper and bag. Check the actual G1 LiDAR topic with `ros2 topic list -t` and `ros2 topic info ... -v` before changing `source_topic`; `/head/points` is only a configurable default. The source simulator may need additional setup/model assets described by its branch README.

## Test matrix on each host

1. `python3 -m unittest discover -s tests -v` (offline report tests).
2. `colcon build --symlink-install --cmake-args -DBUILD_CUDA=OFF` (CPU baseline).
3. `ros2 launch g1_cuda_lidar_benchmark smoke.launch.py backend:=cpu` (synthetic DDS integration, **not** MuJoCo validation).
4. When CUDA and compatible GPU are installed: rebuild with `-DBUILD_CUDA=ON`; run smoke for `naive` and `optimized`. Confirm CPU reference equality.
5. Start upstream `unitree_g1.launch.py`, inspect true LiDAR topic and TF, capture bag and run real three-mode replay benchmark.
6. Collect `collect_system_info.sh` and upstream git commit for each experiment.

## Caveats

- Different MuJoCo or CUDA toolkit versions across operating systems may change the simulated workload or GPU execution. For **CPU vs GPU** speedup measurements, compare modes on the **same machine and same distribution**, using an identical captured bag.
- ROS Humble's `main` G1 feature set may differ from the `jazzy` branch; test the actual installed example and do not assume topic naming.
- Enabling the CUDA backend requires NVTX3 headers and a CUDA Toolkit compatible with each OS, GPU and driver. CPU-only mode must compile without CUDA.
- No ROS 2 or live MuJoCo environment was available during artifact construction; offline checks are not evidence of successful Humble/Jazzy builds.
