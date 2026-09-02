# AMD-UW SynChrono Demo

Running instructions. Why the site is laid out and tuned the way it is, and what the
recording formats contain, is in [DESIGN.md](DESIGN.md); unattended long runs are in
[docs/AUTORUN.md](docs/AUTORUN.md).

**Rank 0 owns no robot, so robots = np - 1.** The sim, the collector launch and the builder
launch must all agree on the count.

## Paths

`CMakeLists.txt` assumes container-absolute paths. Check these on a new checkout:

```text
CHRONO_ROOT        = /home/chrono-user/mountdir/chrono
CHRONO_BUILD_DIR   = ${CHRONO_ROOT}/build
CHRONO_PACKAGE_DIR = /home/chrono-user/mountdir/packages
UW_AMD_DATA_DIR    = ${CMAKE_CURRENT_SOURCE_DIR}/data
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw
cmake -S . -B build -G Ninja -DAMD_UW_ENABLE_ROS2=ON   # once, or after a fresh container
ninja -C build -j4                                     # -j4: more threads have hard-locked this box
```

Rebuild after editing sources with `ninja -C build -j4`; no reconfigure needed.

ROS 2 controllers:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
colcon build --symlink-install --packages-select amd_uw_ros2
source install/setup.bash
```

`--symlink-install` makes Python edits live. If `ros2 launch` cannot find a newly added
launch file, `rm -rf build/amd_uw_ros2 install/amd_uw_ros2` and rebuild.

**ROCm host (HPC Fund)** — add `-DAMD_UW_ENABLE_CUDA=OFF` and run with `--no_sensor`:

```bash
cmake -S . -B build -G Ninja -DAMD_UW_ENABLE_ROS2=ON -DAMD_UW_ENABLE_CUDA=OFF \
      -DCHRONO_ROOT=$HOME/mountdir/chrono -DCHRONO_PACKAGE_DIR=$HOME/mountdir/packages
```

## Run

Three terminals, all sourcing `/opt/ros/humble/setup.bash` and the workspace. The example is
**16 ranks = 15 robots**, which is this workstation's maximum (16 physical cores).

Terminal 1 — collectors:

```bash
cd ~/mountdir/amd-uw/ros2_ws && source install/setup.bash
ros2 launch amd_uw_ros2 robot_controllers.launch.py \
  robot_ids:=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 \
  target_speed_mps:=3.0 switch_radius_m:=2.0 rock_side_offset_m:=2.0
```

Terminal 2 — builders (orbit + arm):

```bash
cd ~/mountdir/amd-uw/ros2_ws && source install/setup.bash
ros2 launch amd_uw_ros2 builder_orbit_controllers.launch.py \
  builder_ids:=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 counter_clockwise:=true
```

Terminal 3 — the sim:

```bash
cd ~/mountdir/amd-uw
mpirun -np 16 -x OMP_NUM_THREADS=1 ./build/demo_SYN_construction \
  --no_sensor --scm_delta 0.035 -e 6000 --perf_log 10 --record_rate 30 --scm_record_rate 10
```

`OMP_NUM_THREADS=1` is deliberate: 16 ranks on 16 cores, one thread each. To watch a rank,
drop `--no_sensor` and add `--vsg 1` (one window per listed rank).

Radii default to the current site (`work_circle_radius_m:=50.0 path_radius_m:=53.0`) and the
station bands derive themselves from the measured slot pitch — do not pass
`station_tolerance_rad`, `station_keep_deadband_m` or `station_keep_max_steering`. Each
controller logs its derived bands at startup; `PINNED` in that line means something is
overriding them.

Scaling the robot count needs no layout change, but note that builders are **not** kept apart
by geometry. `BuilderWallSlotCount` is a flat 200 slots (100 m of lane, ~4.8 sectors at N=15):
builder `n` deliberately walks off the end of its own sector and lays over what `n+1` finished.
Separation is a control problem, handled by `min_builder_gap_m` (9.0 m) in
`builder_orbit_controller.py` — and that fence is fail-open, so a silent neighbour imposes no
constraint. See DESIGN.md.

### Sizing a run

`--scm_delta` decides whether a run fits. Measured **2.55 GB per robot rank at 0.042** and
**3.53 GB at 0.035**; memory scales as `1/delta²`, so per-rank GB is `2.55 × (0.042/delta)²`.

| robots | delta 0.020 | delta 0.035 | delta 0.042 | delta 0.050 |
|---|---|---|---|---|
| 4 | 45 GB | 14 GB | 10.3 GB | 7.2 GB |
| 8 | 90 GB | 28 GB | 20.5 GB | 14.4 GB |
| 15 | 169 GB | 53 GB | 38 GB | 27 GB |
| 20 | 225 GB | 71 GB | 51 GB | 36 GB |

This workstation has 62 GB and 16 physical cores. **15 robots at 0.035 measures 52.9 GB
steady** — the practical ceiling here; 20 robots at 0.035 (71 GB) is over it. Check `free -g`
before launching and run one sim at a time.

MPI slots are **physical cores**, not threads (`nproc` reports 24 because of SMT), so anything
above `-np 16` fails with "not enough slots" unless you add `--map-by hwthread`, which shares
cores and slows the run. Cluster nodes are 128 cores; the 0.020 grid is node-only. See
[batch/README.md](batch/README.md).

## Flags

| flag | default | effect |
|---|---|---|
| `-e, --end_time` | — | sim seconds to run |
| `-s, --step_size` | — | integration step |
| `--scm_delta` | 0.10 | SCM grid spacing; sets the memory footprint |
| `--no_sensor` | off | no camera on rank 0; required without CUDA |
| `--vsg 1,2` | none | MPI ranks that open a window |
| `--perf_log N` | 0 (off) | per-rank cost breakdown every N sim seconds |
| `--settle_time` | — | braked pre-roll before SynChrono starts |
| `--rocks_per_rank N` | hashed 2–6 | pin every rank to N rocks, for reproducible tests |
| `--builder_no_arm` | off | builder without manipulator (cost bisection) |
| `--no_builder` / `--no_build` | off | drop the builder / the build behaviour |
| `--solver`, `--solver_iterations` | BB, 100 | override the solver |
| `--scm_raycast_gpu`, `--scm_gpu_min_hits` | off | GPU ray casting for SCM |
| `--record_root DIR` | `recordings/` | parent for `run_<timestamp>/` |
| `--record_dir DIR` | — | record to exactly this directory |
| `--record_rate` | 60 Hz | pose sample rate |
| `--scm_record_rate` | 10 Hz | deformation sample rate |
| `--no_record` | off | disable recording (it is **on** by default) |

Rank 0 prints the absolute recording path at startup.

## Tools

```bash
# 3D playblast with the run's own meshes (needs pyvista; --movie/--shot are off-screen)
python3 tools/replay_run.py <dir>                             # real time
python3 tools/replay_run.py <dir> --speed 8
python3 tools/replay_run.py <dir> --rank 1,2 --from 90 --to 140
python3 tools/replay_run.py <dir> --no-running-gear           # drop tracks, ~70 fps
python3 tools/replay_run.py <dir> --movie clip.mp4
python3 tools/replay_run.py <dir> --shot look.png --at 300
python3 tools/replay_run.py <dir> --focus "builder/Chassis" --focus-dist 7
python3 tools/replay_run.py <dir> --max-frames 9000           # hold every frame of a long run

# validate and inspect a recording
python3 tools/read_trajectory.py <dir> --check
python3 tools/read_trajectory.py <dir> --rank 1 --bbox 0      # rebuild one frame, print extents
python3 tools/read_scm.py <dir> --check

# regrade the level work pad (overwrites data/terrain/terrain2_graded.png)
python3 tools/make_graded_pad.py --pad-radius 65 --taper-radius 130
```

`replay_run.py` keys: `space` play/pause, `←`/`→` step, `[`/`]` speed, `t` top, `c` free
camera, `f` follow next machine, `r` restart, `q` quit. Camera flies on `ijkl` (like `wasd`),
`u`/`o` up and down.

## Debug topics

```bash
ros2 topic echo /robot_1/egoState        # also: targetPos, vehicle_cmd, arm_status
ros2 topic echo /builder_1/arm_status
```

Force a collector to the next rock, or drive a dump cycle by hand:

```bash
ros2 topic pub --once /robot_1/target_done std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /robot_1/trailer_cmd std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic echo /robot_1/trailer_state
```

`trailer_state` is `[state, bed_angle_rad, tailgate_angle_rad]`; state `0` idle, `1` opening
gate, `2` tilting, `3` dwell, `4` levelling, `5` closing gate, `6` done.

Arm status error codes: `0` none, `1` bad target_index, `2` IK failed / unreachable,
`3` lock failed (fingers closed but rock not close enough), `4` timeout.

## Stray controllers: check this first

`ros2 launch` does **not** take its children down when the launch process is killed. Ctrl-C in
the terminal is fine; killing the launch PID leaves a full set of controllers running,
invisible until they fight the next run.

```bash
# before every run -- expect nothing
pgrep -af "pure_pursuit_controller|manipulator_controller|builder_orbit_controller|builder_arm_controller"

# clean up
pkill -9 -f "pure_pursuit_controller|manipulator_controller|builder_orbit_controller|builder_arm_controller"

# from a script: own process group, kill the group
setsid ros2 launch amd_uw_ros2 robot_controllers.launch.py ... &
kill -TERM -$!
```

The sim reports duplicates on the first offending step. If you see `N publishers on ...
-- expected 1`, stop and clear strays; results until then are not trustworthy. DESIGN.md
describes what duplication looks like when you don't catch it.
