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
cmake -S . -B build -G Ninja -DAMD_UW_ENABLE_ROS2=ON   # configure once (or after a fresh container)
ninja -C build                                          # build the whole project
```

After editing sources, just rebuild (no reconfigure needed):

```bash
ninja -C build
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
  target_speed_mps:=3.0 \
  switch_radius_m:=2.0 \
  rock_side_offset_m:=2.0
```

Terminal 2, start the C++ sim:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw
mpirun -np 3 ./build/demo_SYN_polaris_flat --vsg 1,2
```

### Site layout

Everything is concentric about `(0, 0)`, and each rank owns one ray out from the
centre, evenly spaced (`2*pi*rank/N`). On that ray:

```text
centre ---- work circle (30 m) ---- builder orbit (40 m) ---- collector (50 m) ----> rock line
              yellow ring                cyan ring                green ring
```

So a rank's collector starts directly outside its own builder, on the line joining
the site centre to it, faces radially outward, and its rock line runs away from
the site — the collector drives out to fetch and back inward to the builder it
feeds, and no rock line crosses the build area. Builders sit tangent to their
orbit, so their radial footprint is only the hull width and there is ~3.5 m of
clearance between a collector's trailer and its builder at spawn.

All of this lives in `src/RobotLayout.h` as one source of truth. Note this also
scales past two ranks: the previous layout hard-coded two headings (330 deg and
60 deg) and silently gave every rank after the second the same heading as rank 2.

The rock line runs 30 m to 660 m out from each collector, so its outer end leaves
the 1024 m heightmap; those last few rocks rest on the flat extension strips that
`AddRockLineTerrainExtensions` lays along each ray, not on mapped terrain.

Start the builder orbit controllers with:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
source install/setup.bash
ros2 launch amd_uw_ros2 builder_orbit_controllers.launch.py \
  builder_ids:=1,2 \
  work_circle_radius_m:=30.0 \
  path_radius_m:=40.0 \
  target_speed_mps:=1.0 \
  lookahead_m:=8.0 \
  counter_clockwise:=true
```

Each controller reads `/builder_N/vehicle_state` (`[x, y, yaw, speed]`) and
publishes `/builder_N/vehicle_cmd` (`[steering, throttle, braking]`). A builder
starts braked and holds until its first valid ROS command arrives.

Its hull is **not** pinned while parked, and must not be. Pinning it is
self-consistent only on flat ground; on this heightmap the hull is placed level
over ground that pitches and rolls a few degrees, the suspension then has no way
to absorb the mismatch, and the single-pin track throws shoes and goes NaN within
about a second on every rank. That was invisible for a long time because the orbit
controller releases the hull within ~50 ms — it only showed up on a rank whose
controller was never started. A free hull with brakes on settles onto the real
surface and holds to within a few cm.

### One system per rank

Each physics rank has exactly **one** `ChSystem`, holding that rank's rover,
trailer, rocks, terrain, and builder. This is a hard requirement, not a
preference: Chrono only generates contacts between bodies of the same system, so
a builder in its own system could never touch the rocks it exists to place.

The consequence is that the whole system runs the solver the single-pin track
needs — `ChSolverBB` at 100 iterations with `EULER_IMPLICIT_LINEARIZED`, set once
in `main.cpp` (the default solver lets track shoes drift off the road wheels).
`BuilderRig` deliberately does not touch gravity, solver, timestepper, or terrain:
it does not own that world.

Two ordering rules follow, and breaking either is silent:

- Every `Synchronize` runs before any `Advance`, and the system is stepped exactly
  once per loop iteration. `BuilderRig::Advance` only advances its own subsystems;
  `robot->Advance` is what issues the `DoStepDynamics`.
- The builder is constructed *after* the rover's SolidWorks arm import, which
  leaves the collision system in a state where later bodies never register
  contacts. `BuilderRig` calls `BindAll()` on the shared system to repair that —
  without it the tracks sink through the ground and the gripper passes through
  rocks.

Watch the terrain cost when changing the field map. The rover's mission needs the
full 1024 m `terrain2.bmp`, which is a ~130k-triangle collision mesh, and the
M113's ~130 track shoes query it every step; that narrowphase is ~7.6 of the ~17.9
wall/sim. If it ever needs to come down, the builder only ever occupies the orbit,
so a second smaller patch plus collision families would let its shoes skip the
big mesh.

### Cost diagnostics

`--perf_log <seconds>` prints a per-rank breakdown every N sim seconds: the
*instantaneous* wall/sim for the window (the plain `wall/sim` line is a cumulative
average, which hides when a cost starts growing — it is why an early slowdown here
read as unbounded growth when it had actually plateaued), the wall time in each
loop section, Chrono's per-step timers, and the builder's drive command / speed /
orbit radius / track-shoe spread.

```bash
mpirun -np 3 ./build/demo_SYN_polaris_flat --no_sensor -e 10 --perf_log 0.5
```

`--builder_no_arm` drops the manipulator, to separate arm cost from vehicle cost.

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
3. [x] Integrate with Harry.
4. [ ] [WIP] Move to SCM terrain. 
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
