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
ros2 run amd_uw_ros2 pure_pursuit_controller --ros-args -p robot_id:=1 -p target_speed_mps:=7.0 -p switch_radius_m:=2.0
```

Terminal 3:

```bash
cd ~/mountdir/amd-uw/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amd_uw_ros2 pure_pursuit_controller --ros-args -p robot_id:=2 -p target_speed_mps:=7.0 -p switch_radius_m:=2.0
```

Terminal 4: (To move to the next target)
```bash
ros2 topic pub --once /robot_1/target_done std_msgs/msg/Bool "{data: true}"
```


`targetPos` contains rock centers. The pure-pursuit controller picks the nearest unfinished rock and drives toward a lateral waypoint beside it (`rock_side_offset_m`). As the tractor rear reference point approaches the pickup zone, the controller linearly caps target speed across `pickup_slowdown_offset_m` so it reaches the `switch_radius_m` boundary at `pickup_boundary_speed_mps` instead of entering the circle at cruise speed. The default slowdown band is `10 m` and aims to enter the circle at `2.0 m/s`. It stops and waits when that waypoint is within `switch_radius_m` of the rear reference point and the rock bearing is in the pickup sector (`60..100` or `-100..-60` degrees with the tractor forward axis as `0`). Publish `true` on `/robot_N/target_done` to mark that rock finished and move to the next target after a short zero-steering settle.

## TODO

1. [x] Add ROS controller integration.
2. [x] Stop at rock.
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
