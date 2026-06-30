# AMD-UW SynChrono Demo

## Directory Layout

`CMakeLists.txt` currently assumes absolute Chrono/container paths. If your checkout is arranged differently, edit the path defaults near the top of `CMakeLists.txt`.

```text
/home/chrono-user/mountdir/
|-- amd-uw/                 <- this project repo
|   |-- CMakeLists.txt
|   |-- data/
|   |-- ros2_ws/
|   `-- src/
|
|-- chrono/                 <- CHRONO_ROOT
|   |-- build/              <- CHRONO_BUILD_DIR
|   `-- data/
|
`-- packages/               <- CHRONO_PACKAGE_DIR
    `-- optix/              <- OPTIX_INSTALL_DIR
```

```text
CMake defaults to check/edit:
CHRONO_ROOT        = /home/chrono-user/mountdir/chrono
CHRONO_BUILD_DIR   = ${CHRONO_ROOT}/build
CHRONO_PACKAGE_DIR = /home/chrono-user/mountdir/packages
UW_AMD_DATA_DIR    = ${CMAKE_CURRENT_SOURCE_DIR}/data
```

## Build

Build the C++ sim with ROS2 support from a ROS2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw
cmake -S . -B build -DAMD_UW_ENABLE_ROS2=ON
ninja 
```

Build the ROS2 Python controllers:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
colcon build --symlink-install --packages-select amd_uw_ros2
source install/setup.bash
```

If `ros2 launch` cannot find a newly added launch file after a failed build,
remove the stale package build/install folders and rebuild:

```bash
cd ~/mountdir/amd-uw/ros2_ws
rm -rf build/amd_uw_ros2 install/amd_uw_ros2
colcon build --symlink-install --packages-select amd_uw_ros2
source install/setup.bash
```

## Run ROS Demo

Terminal 1, start the ROS controllers for robot ranks 1 and 2:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
source install/setup.bash
ros2 launch amd_uw_ros2 robot_controllers.launch.py \
  robot_ids:=1,2 \
  target_speed_mps:=7.0 \
  switch_radius_m:=2.0
```

Terminal 2, start the C++ sim:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw
mpirun -np 3 ./build/demo_SYN_polaris_flat --vsg 1,2
```

Rank layout:

```text
rank 0 = global sensor/visualization rank
rank 1 = robot 1
rank 2 = robot 2
```

Terminal 3, optional status/debug commands:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /robot_1/egoState
ros2 topic echo /robot_1/targetPos
ros2 topic echo /robot_1/vehicle_cmd
ros2 topic echo /robot_1/arm_status
ros2 topic echo /robot_2/arm_status
```

Manual target completion, if you need to force a robot to move to the next rock:

```bash
ros2 topic pub --once /robot_1/target_done std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /robot_2/target_done std_msgs/msg/Bool "{data: true}"
```

`targetPos` contains rock centers. The pure-pursuit controller picks the nearest unfinished rock and drives toward a lateral waypoint beside it (`rock_side_offset_m`). As the tractor rear reference point approaches the pickup zone, the controller linearly caps target speed across `pickup_slowdown_offset_m` so it reaches the `switch_radius_m` boundary at `pickup_boundary_speed_mps` instead of entering the circle at cruise speed. The default slowdown band is `10 m` and aims to enter the circle at `2.0 m/s`. It stops and waits when that waypoint is within `switch_radius_m` of the rear reference point and the rock bearing is in the pickup sector (`60..100` or `-100..-60` degrees with the tractor forward axis as `0`). Publish `true` on `/robot_N/target_done` to mark that rock finished and move to the next target after a short zero-steering settle.

When the rover stops at a pickup target, the drive controller publishes
`/robot_N/pickup_request`; the manipulator controller relays that to the C++ sim
on `/robot_N/arm_cmd`. The C++ arm publishes `/robot_N/arm_status`, and the
manipulator controller publishes `/robot_N/target_done=true` after the arm
reports completion or, by default, after a failed target is skipped.

Arm status error codes:

```text
0 = none
1 = bad target_index
2 = IK failed / unreachable
3 = lock failed / fingers closed but rock was not close enough
4 = timeout
```

## TODO

1. [x] Add ROS controller integration.
2. [x] Stop at rock.
3. [ ] Integrate with Harry.
4. [ ] Move to SCM terrain.
5. [x] ~~Explore a PyChrono wrapper for SynChrono~~. Scrapped.
6. [ ] Scale to many vehicles and rocks.

## Refactoring Note

- [x] Keep the current demo single-file while it remains one compact executable.
- [x] Split robot rig setup and per-step robot updates into their own module.
- [x] Split rock field generation into its own module.
- [x] Split custom SynChrono agents into their own module.
- [x] Continue refactoring along domain boundaries when scale-up requires it: terrain/world setup, ROS controller drivers, robot arm, and per-robot sensors.

## Side Quest

- [ ] Explore texture support in Chrono.
