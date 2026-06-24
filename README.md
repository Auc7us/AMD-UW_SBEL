# AMD-UW SynChrono Demo

## Run ROS Demo

Terminal 1:

```bash
cd ~/mountdir/amd-uw/build
source /opt/ros/humble/setup.bash
mpirun -np 3 ./demo_SYN_polaris_flat --vsg 1,2
```

Terminal 2:

```bash
cd ~/mountdir/amd-uw/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amd_uw_ros2 pure_pursuit_controller --ros-args -p robot_id:=1 -p target_speed_mps:=10.0 -p switch_radius_m:=2.0
```

Terminal 3:

```bash
cd ~/mountdir/amd-uw/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amd_uw_ros2 pure_pursuit_controller --ros-args -p robot_id:=2 -p target_speed_mps:=10.0 -p switch_radius_m:=2.0
```

`targetPos` contains rock centers. The pure-pursuit controller picks the nearest unfinished rock, drives toward a lateral waypoint beside it (`rock_side_offset_m`, default `2.0 m`), and marks it finished when the tractor ego position is within `switch_radius_m` of that waypoint.

## TODO

1. [x] Add ROS controller integration.
2. [ ] Stop at rock.
3. [ ] Move to SCM terrain.
4. [ ] Explore a PyChrono wrapper for SynChrono.
5. [ ] Integrate with Harry.
6. [ ] Scale to many vehicles and rocks.

## Refactoring Note

- [x] Keep the current demo single-file while it remains one compact executable.
- [x] Split robot rig setup and per-step robot updates into their own module.
- [x] Split rock field generation into its own module.
- [x] Split custom SynChrono agents into their own module.
- [x] Continue refactoring along domain boundaries when scale-up requires it: terrain/world setup, ROS controller drivers, robot arm, and per-robot sensors.

## Side Quest

- [ ] Explore texture support in Chrono.
