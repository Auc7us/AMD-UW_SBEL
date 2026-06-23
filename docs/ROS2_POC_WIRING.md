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

## C++ Integration Target

Add a future `RosControllerDriver` owned by `RobotRig`.

Expected behavior:

1. Subscribe to `/robot_N/vehicle_cmd`.
2. Clamp command values.
3. Store the latest command.
4. Return Chrono `DriverInputs` from that command.
5. Publish `/robot_N/state` each simulation tick.

`RobotRig::Synchronize()` should eventually ask this driver for inputs instead of the current interactive driver.

## Later Terminal-Pose Planner

The later pickup planner should replace `simple_goal_controller` without changing the C++ command topic.

Future planner shape:

1. Receive rock target from C++.
2. Sample valid terminal robot poses around the rock.
3. Require rock to end in the rear-arm reachable side sector: `[80, 120]` or `[-120, -80]` degrees in robot frame.
4. Reject terminal poses where the robot or trailer collides with the rock.
5. Plan a feasible tractor-trailer approach path.
6. Publish the same `[steering, throttle, brake]` command.
