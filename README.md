# AMD-UW SynChrono Demo

## Run ROS Demo

Terminal 1:

```bash
cd ~/mountdir/amd-uw/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run amd_uw_ros2 constant_speed_controller --ros-args -p robot_id:=1 -p target_speed_mps:=1.0
```

Terminal 2:

```bash
cd ~/mountdir/amd-uw/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amd_uw_ros2 constant_speed_controller --ros-args -p robot_id:=2 -p target_speed_mps:=1.0
```

Terminal 3:

```bash
cd ~/mountdir/amd-uw
source /opt/ros/humble/setup.bash
cmake -S . -B build -DAMD_UW_ENABLE_ROS2=ON
cmake --build build -j2
mpirun -np 3 ./build/demo_SYN_polaris_flat --vsg 1,2
```

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
