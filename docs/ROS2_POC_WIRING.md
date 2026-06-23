# ROS2 POC Wiring

This is a first-pass ROS2 Humble controller package for proving the control loop before the terminal pickup planner exists.

## Build

From a ROS2 Humble environment:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run one controller:

```bash
ros2 run amd_uw_ros2 simple_goal_controller --ros-args -p robot_id:=1 -p target_x:=8.0 -p target_y:=0.0
```

Run the speed-only controller:

```bash
ros2 run amd_uw_ros2 constant_speed_controller --ros-args -p robot_id:=1 -p target_speed_mps:=1.0
```

## Temporary Topic Contract

For robot rank `N`, the C++ sim should publish:

```text
/robot_N/state          std_msgs/Float64MultiArray
data = [x, y, yaw, speed_mps]
```

The ROS node publishes:

```text
/robot_N/vehicle_cmd    std_msgs/Float64MultiArray
data = [steering, throttle, brake]
```

Where:

```text
steering in [-1, 1]
throttle in [0, 1]
brake in [0, 1]
```

Optional target override:

```text
/robot_N/target_pose    geometry_msgs/Pose2D
```

## Speed-Only POC

`constant_speed_controller` ignores steering for now and publishes:

```text
data = [0.0, throttle, brake]
```

It ramps the target speed, throttle, and brake so commands change gradually. This is intended as the first closed-loop vehicle-control test before adding steering or terminal-pose planning.

## C++ Integration Target

`RosControllerDriver` is owned by `RobotRig` when CMake finds ROS2 `rclcpp` and `std_msgs`.

Expected behavior:

1. Subscribe to `/robot_N/vehicle_cmd`.
2. Clamp command values.
3. Store the latest command.
4. Return Chrono `DriverInputs` from that command.
5. Publish `/robot_N/state` each simulation tick.

`RobotRig::Synchronize()` asks this driver for inputs instead of the interactive driver when `AMD_UW_ENABLE_ROS2` is enabled.

If ROS2 is not found at CMake configure time, the C++ sim falls back to the interactive driver.

## Full POC Run Order

Terminal 1, robot rank 1 controller:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run amd_uw_ros2 constant_speed_controller --ros-args -p robot_id:=1 -p target_speed_mps:=1.0
```

Terminal 2, robot rank 2 controller:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
source install/setup.bash
ros2 run amd_uw_ros2 constant_speed_controller --ros-args -p robot_id:=2 -p target_speed_mps:=1.0
```

Terminal 3, C++ sim:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw
cmake -S . -B build -DAMD_UW_ENABLE_ROS2=ON
cmake --build build -j2
mpirun -np 3 ./build/demo_SYN_polaris_flat
```

Rank layout:

```text
rank 0 = global sensor/visualization rank
rank 1 = robot 1
rank 2 = robot 2
```

Terminal 4, optional:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /robot_1/state
ros2 topic echo /robot_1/vehicle_cmd
```

## Later Terminal-Pose Planner

The later pickup planner should replace `simple_goal_controller` without changing the C++ command topic.

Future planner shape:

1. Receive rock target from C++.
2. Sample valid terminal robot poses around the rock.
3. Require rock to end in the rear-arm reachable side sector: `[80, 120]` or `[-120, -80]` degrees in robot frame.
4. Reject terminal poses where the robot or trailer collides with the rock.
5. Plan a feasible tractor-trailer approach path.
6. Publish the same `[steering, throttle, brake]` command.
