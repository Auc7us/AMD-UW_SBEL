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

Run the target-position steering controller:

```bash
ros2 run amd_uw_ros2 pure_pursuit_controller --ros-args -p robot_id:=1 -p target_speed_mps:=1.0 -p switch_radius_m:=3.0
```

Launch drive and manipulator controllers together:

```bash
ros2 launch amd_uw_ros2 robot_controllers.launch.py robot_ids:=1,2 target_speed_mps:=1.0 switch_radius_m:=3.0
```

## Temporary Topic Contract

For robot rank `N`, the C++ sim publishes once per simulation loop:

```text
/robot_N/egoState       std_msgs/Float64MultiArray
data = [x, y, yaw, speed_mps]

/robot_N/targetPos      std_msgs/Float64MultiArray
data = [rock0_x, rock0_y, rock1_x, rock1_y, ...]
```

The ROS node publishes:

```text
/robot_N/vehicle_cmd    std_msgs/Float64MultiArray
data = [steering, throttle, brake]

/robot_N/pickup_request std_msgs/Float64MultiArray
data = [target_index, rock_x_global, rock_y_global]
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

Manual target completion:

```text
/robot_N/target_done    std_msgs/Bool
data = true
```

Manipulator coordination:

```text
/robot_N/arm_cmd        std_msgs/Float64MultiArray
data = [command_seq, target_index, rock_x_global, rock_y_global]

/robot_N/arm_status     std_msgs/Float64MultiArray
data = [command_seq, state, target_index, success, error_code]
```

Arm status state codes are `0=idle`, `1=busy`, `2=done`, and `3=failed`.
Error codes are `0=none`, `1=bad_target`, `2=ik_failed`, `3=lock_failed`, and
`4=timeout`.

## Speed-Only POC

`constant_speed_controller` ignores steering for now and publishes:

```text
data = [0.0, throttle, brake]
```

It ramps the target speed, throttle, and brake so commands change gradually. This is intended as the first closed-loop vehicle-control test before adding steering or terminal-pose planning.

## Pure-Pursuit Target POC

`pure_pursuit_controller` subscribes to `/robot_N/egoState`, `/robot_N/targetPos`, and `/robot_N/target_done`.

It picks the nearest unfinished `[x, y]` rock target and offsets the drive waypoint to one side of that rock by `rock_side_offset_m`. As the tractor rear reference point approaches the pickup zone, it linearly caps target speed across `pickup_slowdown_offset_m`, using `pickup_min_approach_speed_mps` and `pickup_boundary_speed_mps` as the low-speed limits, so the robot reaches the `switch_radius_m` boundary slowly instead of entering the circle at cruise speed. The default slowdown band is `10 m` and aims to enter the circle at `2.0 m/s`. It stops and waits when the drive waypoint is within `switch_radius_m` meters of the rear reference point and the rock bearing from that rear reference is in either pickup sector: `60..100` degrees or `-100..-60` degrees, with the tractor forward axis as `0` degrees. Publishing `true` on `/robot_N/target_done` marks the waiting target complete and lets the controller pick the nearest remaining rock after a short zero-steering settle. It publishes:

```text
data = [steering, throttle, brake]
```

While waiting, it publishes `/robot_N/pickup_request` at `pickup_request_rate_hz`
(default `1 Hz`) so the manipulator controller can start even if launched after
the rover has already stopped.

The side is chosen once per rock from the two lateral waypoints perpendicular to the robot-to-rock approach line, preferring the side that needs the smaller heading change. Steering is normalized from a pure-pursuit steering angle using `wheelbase_m` and `max_steering_angle_rad`. Speed uses the same ramped speed controller as the speed-only node.

## Manipulator Controller

`manipulator_controller` subscribes to `/robot_N/pickup_request`, publishes one
deduplicated `/robot_N/arm_cmd` per target, listens for `/robot_N/arm_status`,
and publishes `/robot_N/target_done=true` after the C++ arm executor reports
success. With the default `skip_failed_targets=true`, failed arm picks are also
marked done so the drive controller can continue to the next rock.

Run it beside the drive controller:

```bash
ros2 run amd_uw_ros2 manipulator_controller --ros-args -p robot_id:=1
```

Or start both the drive and manipulator controllers for each robot with:

```bash
ros2 launch amd_uw_ros2 robot_controllers.launch.py robot_ids:=1,2
```

## C++ Integration Target

`RosControllerDriver` is owned by `RobotRig` when CMake finds ROS2 `rclcpp` and `std_msgs`.

Expected behavior:

1. Subscribe to `/robot_N/vehicle_cmd`.
2. Clamp command values.
3. Store the latest command.
4. Return Chrono `DriverInputs` from that command.
5. Publish `/robot_N/egoState` and `/robot_N/targetPos` each simulation tick.
6. Subscribe to `/robot_N/arm_cmd`, execute the arm pick/place in the C++ sim,
   and publish `/robot_N/arm_status`.

`RobotRig::Synchronize()` asks this driver for inputs instead of the interactive driver when `AMD_UW_ENABLE_ROS2` is enabled.

If ROS2 is not found at CMake configure time, the C++ sim falls back to the interactive driver.

## Full POC Run Order

Terminal 1, ROS controllers for robot ranks 1 and 2:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch amd_uw_ros2 robot_controllers.launch.py robot_ids:=1,2 target_speed_mps:=1.0 switch_radius_m:=3.0
```

Terminal 2, C++ sim:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw
cmake -S . -B build -DAMD_UW_ENABLE_ROS2=ON
ninja -C build -j2
mpirun -np 3 ./build/demo_SYN_polaris_flat
```

Rank layout:

```text
rank 0 = global sensor/visualization rank
rank 1 = robot 1
rank 2 = robot 2
```

Terminal 3, optional:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /robot_1/egoState
ros2 topic echo /robot_1/targetPos
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
