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

On a **ROCm host** (HPC Fund) add `-DAMD_UW_ENABLE_CUDA=OFF`. Chrono::Sensor's OptiX ray
tracer is CUDA-only, so the default link line pulls in `/usr/local/cuda/...` — which does
not exist there at all, and the link fails on `libcudart_static.a` before anything else
is even attempted. With the switch off the demo links against a CUDA-free
`libChrono_sensor`; rank 0's camera then has no ray tracer behind it, so run with
`--no_sensor`:

```bash
cmake -S . -B build -G Ninja -DAMD_UW_ENABLE_ROS2=ON -DAMD_UW_ENABLE_CUDA=OFF \
      -DCHRONO_ROOT=$HOME/mountdir/chrono -DCHRONO_PACKAGE_DIR=$HOME/mountdir/packages
ninja -C build
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

Shown here at **5 robot ranks**. Rank 0 is the sensor/visualization rank and owns no
robot, so `num_robot_ranks = np - 1` -- five robots means `-np 6`, and the rank ids the
controllers address are `1..5`. Every rank parses the same command line and derives the
layout from that one count, so the three commands below have to agree on it.

Terminal 1, start the collector controllers for robot ranks 1-5:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
source install/setup.bash
ros2 launch amd_uw_ros2 robot_controllers.launch.py \
  robot_ids:=1,2,3,4,5 \
  target_speed_mps:=5.0 \
  switch_radius_m:=2.0 \
  rock_side_offset_m:=2.0
```

Terminal 2, start the builder orbit + arm controllers for the same ranks. Each rank owns
one collector *and* one builder, so `builder_ids` mirrors `robot_ids` -- a builder without
its controller starts braked and holds slot 0 for the whole run:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
source install/setup.bash
ros2 launch amd_uw_ros2 builder_orbit_controllers.launch.py \
  builder_ids:=1,2,3,4,5 \
  work_circle_radius_m:=30.0 \
  path_radius_m:=33.0 \
  counter_clockwise:=true
```

Terminal 3, start the C++ sim:

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw
mpirun -np 6 ./build/demo_SYN_construction --vsg 1
```

`--vsg` takes the MPI ranks that should open a window, one window each, so
`--vsg 1,2,3,4,5` gives five chase cameras on one machine. Pick the one rank you want to
watch unless you actually want all of them.

### Site layout

Everything is concentric about `(0, 0)`, and each rank owns one ray out from the
centre, evenly spaced (`2*pi*rank/N`). On that ray:

```text
centre ---- work circle (30 m) ---- builder orbit (33 m) ---- collector (37 m) ----> rock line
              yellow ring                cyan ring                green ring
```

The two gaps are 3 m inboard and 4 m outboard, both inside the 2.78–4.44 m band the
2x-scaled builder arm proved in service, so one lane serves both the wall and the drop
pile. 35/40 put both gaps at 5 m and the builder had to reposition for every pick.
Rovers are *placed* further out still, at 42 m, so a spawning trailer does not land on
its own builder; only the placement reads that radius.

So a rank's collector starts directly outside its own builder, on the line joining
the site centre to it, faces radially outward, and its rock line runs away from
the site — the collector drives out to fetch and back inward to the builder it
feeds, and no rock line crosses the build area. Builders sit tangent to their
orbit, so their radial footprint is only the hull width and there is ~3.5 m of
clearance between a collector's trailer and its builder at spawn.

All of this lives in `src/RobotLayout.h` as one source of truth. Note this also
scales past two ranks: the previous layout hard-coded two headings (330 deg and
60 deg) and silently gave every rank after the second the same heading as rank 2.

The rock line runs 30 m to 660 m out from each collector, so its outer end can leave
the 1024 m heightmap. Nothing needs stitching there: SCM's patch extends to infinity in
the x-y plane, and outside the initialized area a node simply takes the height of the
nearest initialized one, so a rock past the edge rests on flat ground at the right
elevation. (The rigid build used to lay explicit extension strips for this.)

### What the builder does

The builder **builds**, at its own pace. Its course, its station and its rate are its
own; the only thing it takes from its collector is rock. One wall slot per iteration:

1. The sim publishes `/builder_N/pick_target` with `ready=1` — the hull is parked, slot
   `k` is unlaid, and a rock is within the arm's envelope.
2. `builder_arm_controller` solves grab and place IK in the arm base frame and sends one
   12-element `/builder_N/arm_cmd`.
3. `LrvArm`'s pick/place machine runs it: approach, close on contact, lock, lift, swing
   180°, settle, release, stow.
4. The sim re-fixes the laid rock and advances the station angle by one slot.
5. `builder_orbit_controller` creeps the ~1 m and parks. `ready` goes high for slot `k+1`.

The geometry is all derived from the slot index `k` in `src/RobotLayout.h`, so the arm,
the drive station and the feedstock cannot disagree:

| | radius | angle |
|---|---|---|
| wall slot `k` | 30.0 | `ray + k·pitch` |
| station for `k` | 33.0 | `ray + k·pitch + arm_lead` |
| seed heap | 36.6 | heap-centre slot angle, **no** `arm_lead` |
| collector load `c` | 37.0 | `ray + HarvestDropSlot(c)·pitch` |

`pitch = 0.9 m / 30 m`, so one slot is 0.9 m of course and 0.99 m of lane. `arm_lead =
atan(2.5/33)` exists because the arm mounts 2.5 m *back* along a tangentially parked
hull; adding it to the station puts the *arm base*, not the hull origin, radially
opposite the slot it serves. That is why the heap must **not** also carry it — doing so
pushed every heap to 5.62 m from the arm base, past the 5.2 m guard, and no rock was
reachable. `main.cpp` prints a reach audit at startup so that class of error shows
up at t=0 rather than after the machine has driven to its first station. Measured at
4 ranks: seed heap 3.54–4.37 m, collector load 3.91–4.04 m, wall slot 3.09 m, all
inside the 2.0–5.2 m envelope.

#### How many rocks a rank gets

**2 to 6, drawn per rank**, not a site-wide 2. `RocksPerRank(rank_index)` in
`src/RobotLayout.h` is the single source of truth, and `--rocks_per_rank N` pins every
rank to `N` for a reproducible test. It is a pure integer hash of the rank index rather
than anything drawn from `<random>` or carried in `RockFieldConfig`, because three
separate places have to arrive at the same number in *different processes*:

- `RockField` spawns that many bodies on the owning rank;
- `SynRockAgent` sizes the sensor rank's zombie pool for a rank it knows nothing about
  except its id -- undersize it and the tail of that rank's rocks is simply invisible in
  the global camera;
- `HarvestDropSlot` advances the collector's lane by one load's worth of **wall slots**
  per cycle, which is what keeps a delivered pile inside the arm's reach of the station
  the builder is already driving to.

That third one is why a bigger load is not just "more rocks". A load is tipped out in one
place and the builder then creeps one slot per rock it lays, so with `L` rocks the far end
of the run is `L-1` slots of lane from the pile. At `L=6` the reach audit measured 6.54 m
against a 5.2 m envelope -- the builder would have parked at its slot and refused the pile
beside it. So the load is now dropped at the **middle** of the run of slots it serves
(`HarvestLoadCenterSlot`), exactly as the seed heap has always been centred on the run
*it* serves. That halves the lever arm: 4.70 m at `L=6`, and it also tightens `L=2` from
4.04 m to 3.94 m.

Note the sector cap. `BuilderWallSlotCount` gives each builder 70% of its `2*pi/N`
sector, which at 15 builders is **9 wall slots** -- six of them served by the seed heap.
A 15-builder site is therefore build-limited, not feedstock-limited: each builder lays
its nine rocks and stops while its collector keeps harvesting. Widening that means
shrinking `wall_slot_pitch_m` or accepting neighbouring builders in each other's lane.

#### Where the rock comes from

There is **one** seed heap of six rocks per builder, and that is all the site places.
It exists only to give the builder something to lay during its collector's first
outbound leg, which is minutes of sim long. Everything after it is delivered.

That works because the harvest lane advances in **wall slots**, not in a round number
of degrees. Load `c` is dropped at slot `HarvestDropSlot(c) = 6 + 2c` — exactly where
the builder's wall will have reached, having laid the six seed rocks and the two rocks
of every earlier load. So each pile lands within reach of the station the builder is
already driving to, and the builder clears each pile off the collector circle before
the collector comes back to that stretch of it. The step used to be a flat 30°, which
at 0.03 rad/slot is 17 slots — a two-rock load every 17 slots leaves a wall that is 88%
gaps and every pile 15 m of arc from the builder meant to eat it.

The arm bridge does not index its feedstock. It takes the **nearest un-consumed rock
inside the envelope**, from the seed heap and every delivered load pooled together. That
has to be a search rather than a lookup, because a load is tipped out of a moving
trailer and lands where it lands; running both sources through one rule also means the
changeover from heap to delivery needs no handling at all. The hull's station is pulled
half way toward the rock on offer (clamped to ±2 slots), so a pile that lands a metre or
two off its nominal slot is still worked rather than stared at.

**Every rock is fixed except the one in the gripper.** Seed rocks are created
`SetFixed(true)` with collision on; delivered rocks are frozen the same way once they
come to rest. `LrvArm::TryLockRock` unfixes exactly one when the fingers make contact,
and the arm bridge re-fixes it once laid — it is part of the wall then, and must not be
nudged by the next stone landing beside it. Delivered rocks used to be *released* again
when a builder came within 3 m or pushed hard enough on them; both triggers fire exactly
when the arm is trying to take hold, so the rock would be unfixed and rolling downhill at
the worst possible moment. `UpdateRockCollisionActivation` also skips fixed rocks, or the
distance test would switch collision off on the pile the gripper is about to close on.

Start both halves together (they are a pair; the drive half alone holds slot 0 forever):

```bash
source /opt/ros/humble/setup.bash
cd ~/mountdir/amd-uw/ros2_ws
source install/setup.bash
ros2 launch amd_uw_ros2 builder_orbit_controllers.launch.py \
  builder_ids:=1,2,3,4,5 \
  work_circle_radius_m:=30.0 \
  path_radius_m:=33.0 \
  counter_clockwise:=true
```

The orbit controller reads `/builder_N/vehicle_state` (`[x, y, yaw, speed]`) and
publishes `/builder_N/vehicle_cmd` (`[steering, throttle, braking]`). A builder starts
braked and holds until its first valid ROS command arrives.

Its hull is held by a **virtual anchor** while parked, and released the moment it is
asked to move — full brake with zero throttle is the park signal, which is exactly what
the orbit controller publishes on station, so no extra topic is needed. A braked M113
does not stay put: measured here, it creeps at 0.22–0.27 m/s on full brake, so `m_parked`
never went true and the arm was never offered a pick. The anchor is a spring-damper
*force* on the chassis (5e4 N/m, 4.5e4 N·s/m, plus a yaw pair), horizontal only —
vertical is the suspension's job.

It is a force and not a constraint for a reason that must not be lost. `SetFixed(true)`
does hold the hull, and it **destroys the track**: it removes the body from the solve, so
its velocity goes to zero in one step while ~130 shoes are still solving against the
velocity it had last step. Bisected at 3 ranks with no controllers: pinning on brake died
at t=2.115, pinning only below 0.02 m/s died at t=6.085 (the velocity gate was a
hypothesis, and it was wrong — it only delayed the failure), and `--no_hull_park` ran
clean past t=21. A force has no velocity discontinuity, so it holds station without
shocking the chain.

Seating the hull on a *fitted terrain plane* (`SeatBuilderOnTerrainPlane`, worst-case
error 0.02–0.08 m against 0.24–0.33 m for a single-probe level placement) is still
needed, and still done — that is about not injecting strain at t=0, and it is orthogonal
to how the hull is held once parked.

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

### Terrain: deformable SCM

The ground is `SCMTerrain` — Bekker-Wong deformable soil — over the same 1024 x 1024 m
`terrain2_graded.png` heightmap the rigid build used, at **0.10 m** grid spacing. Soil
parameters and the resolution are the `terrain_scm_*` / `scm_*` constants at the top of
`main.cpp`.

Read the cost before changing the resolution:

| | at 0.10 m |
|---|---|
| grid nodes | 10241 x 10241 = **104.9 M** |
| undeformed heights (dense `ChMatrixDynamic<double>`) | **839 MB per rank** |
| visualization mesh (verts + normals + UV + colour + 209.7 M faces) | **~13.5 GB per rank** |

Both scale as 1/delta^2, so 0.05 m is 4x each figure. The height matrix is allocated
whether or not a node is ever touched.

The visualization mesh is therefore built **only on ranks that actually render** —
a rank with its own `--vsg` window, or rank 0 when its camera is enabled. A headless rank
pays the 839 MB and nothing more. Physics is unaffected either way: SCM works from the
height matrix and the active domains, not from the mesh.

**Active domains are not optional.** SCM only casts rays from grid nodes inside some
active domain, so a body outside every domain gets no soil reaction at all and sinks
straight through the terrain. Four places register them, and all four are needed:

- `RobotRig::InitializeOnTerrain` — the collector's and trailer's wheel spindles, and the
  cycle-0 rocks. Before `Settle()`, or the settle steps run against the whole grid.
- `RobotRig::StartNextHarvestCycle` — each later cycle's rocks as they spawn.
- `main.cpp`, after the builder is built — **one** domain on the M113 hull rather than ~130
  on the shoes (a domain only selects which nodes cast rays; the rays hit whatever shape is
  above them, so one box over the footprint covers every shoe in it), plus one per seed rock.
- `main.cpp`, on ranks that own no robot — a degenerate 1 cm domain on a marker body. Rank 0
  steps the system too, and `SCMLoader::SetupInitial()` only installs Chrono's default
  whole-system domain when a visualization mesh was requested, so a headless rank 0 would
  otherwise index an empty domain list behind a release-disabled assert.

For reference, the rigid terrain this replaced was a ~130k-triangle collision mesh that the
M113's ~130 track shoes queried every step: ~7.6 of the ~17.9 wall/sim. SCM moves that cost
from Bullet's narrowphase to ray casting over the active domains, so it scales with how much
machine is on the ground, not with how big the map is.

### Recording poses for offline rendering

**Recording is on by default.** Every run writes the world pose of every moving body on
every physics rank, at `--record_rate` Hz (default 60), for re-rendering in Blender with
better meshes than the sim carries. A run nobody can re-render afterwards is a run that
has to be repeated, and at this terrain size a repeat is expensive.

Runs are named by timestamp under `--record_root` (default `recordings/`, relative to the
working directory, gitignored):

```text
recordings/run_20260821_181530/
```

Rank 0 stamps that name once and broadcasts it over MPI. Every rank deriving it from its
own clock would scatter one run across as many directories as there are ranks the moment
the second ticked over during startup — and startup here is seconds of heightmap
resampling, so it would tick.

| flag | effect |
|---|---|
| *(nothing)* | record to `recordings/run_<timestamp>` at 60 Hz |
| `--record_root /data` | record to `/data/run_<timestamp>` |
| `--record_dir /data/run1` | record to exactly that directory |
| `--no_record` | do not record |

The absolute path in use is printed by rank 0 at startup.

```bash
mpirun -np 16 ./build/demo_SYN_construction                  # -> recordings/run_<timestamp>
mpirun -np 16 ./build/demo_SYN_construction --record_dir /data/run1 --record_rate 60
python3 tools/read_trajectory.py /data/run1 --check          # validate
python3 tools/read_trajectory.py /data/run1 --rank 1 --list  # what was recorded
python3 tools/read_trajectory.py /data/run1 --rank 1 --npz rank1.npz
python3 tools/read_trajectory.py /data/run1 --rank 1 --bbox 0   # rebuild a frame
```

`--bbox` is the check worth running before writing any Blender importer: it rebuilds one
frame from the files alone -- `body_pose * shape_frame * shape_aabb` -- and prints each
group's world extent. Getting either convention wrong does not error, it just produces a
wrong scene, and the wrongness is visible here as a number. A correct rebuild reads:

```text
builder    size=  2.705 x   5.407 x  1.146 m     <- M113 hull is 2.686 m wide, 5.4 m long
collector  size=  3.511 x   2.496 x  2.070 m
rock       size= 49.723 x  18.025 x  1.863 m     <- the group, i.e. the whole rock line
```

Wheels folded into the hull (the centre-of-mass mistake) collapses the collector's
extent; ignoring the shape frames stacks the M113's road wheels on its centreline.

#### Replaying a recording in 3D, without Blender

`tools/replay_run.py` is a previz playblast: it opens the recording in a real 3D window
using the run's OWN meshes, played back on the wall clock, orbitable and scrubbable. The
scene builds itself -- the object manifest names every mesh file and its local frame, so
there is nothing to import and no scene to maintain.

```bash
python3 tools/replay_run.py ~/mountdir/recordings/run16              # real time
python3 tools/replay_run.py <dir> --speed 8                          # 8x
python3 tools/replay_run.py <dir> --rank 1,2 --from 90 --to 140      # one sector, one cycle
python3 tools/replay_run.py <dir> --no-running-gear                  # drop tracks, faster
python3 tools/replay_run.py <dir> --boxes                            # bounding boxes, faster
python3 tools/replay_run.py <dir> --movie clip.mp4                   # off-screen to mp4
python3 tools/replay_run.py <dir> --shot look.png --at 300           # one frame
python3 tools/replay_run.py <dir> --focus "builder/Chassis" --focus-dist 7   # close on one machine
python3 tools/replay_run.py <dir> --max-frames 9000                  # hold every frame of a long run
```

Keys: `space` play/pause, `<-` `->` step a frame, `[` `]` speed, `t` top, `c` free camera,
`f` follow the next machine, `r` restart, `q` quit.

The camera flies on `ijkl`, laid out the way `wasd` is: `i`/`k` forward and back, `j`/`l`
strafe, `u`/`o` up and down, and pyvista's own `up`/`down` zoom. Movement is in the GROUND
PLANE rather than along the view vector -- the site is looked at from 21 degrees above, so
flying along the view drives into the dirt after a few presses, and gliding over the
terrain with height on its own keys is what gets you across a 200 m work area. Each press
covers a fixed fraction of the distance to whatever the camera is pointed at, so it feels
the same framing the whole site or one machine, and holding a key auto-repeats. Any of them
drops `f` follow, since a followed camera is re-aimed every frame.

The arrows are the timeline and nothing else, and stepping pauses playback -- stepping into
a running clock only fights it. `i` used to frame the iso view, which is what `c` already
does, so the letter went to the camera.

Playback is 1:1 with the recording by default -- `--fps` defaults to the rate in the file,
so every recorded frame is played and nothing is resampled. A 60 Hz recording is therefore
real time at 60 fps, and `--movie` writes it at exactly that: 1800 recorded frames come out
as a 30.0 s, 60.00 fps video.

`--max-frames` (default 3000) is a cap on how many frames are HELD IN MEMORY, not a frame
rate: every body's pose for every frame is resident, about 22 kB per frame for a 15-rank
run. A recording longer than the cap is resampled onto that many frames and plays coarser
than it was recorded -- 139 s at 60 Hz is 8346 frames, so the default plays 21.6 of every
60. The tool prints the flag value that would play every frame; `--from`/`--to` is the
other way out, and costs nothing in fidelity.

Speed runs off the WALL CLOCK: each tick advances by however much sim time has actually
elapsed, so `[` and `]` take effect immediately and a scene too heavy to keep up drops
frames rather than sliding into slow motion. That matters here -- pushing 816 bodies'
poses is ~88 ms a frame against ~12 ms to render them, so a four-rank run with running
gear has a ~10 fps ceiling and 1x playback is dropping five frames in six. It is honest
about it: the clock in the HUD tracks real time either way. `--no-running-gear` (120
bodies) clears 70 fps.

Needs `pyvista` (`pip install pyvista imageio imageio-ffmpeg`), and a display for the
interactive window -- `--movie` and `--shot` render off-screen and need neither.

Lighting is one white sun plus a dim white fill, deliberately not VTK's `enable_lightkit()`,
whose key light is warm by default (warmth 0.6 against 0.5 neutral) and tints neutral grey
regolith olive -- the Moon came out looking like desert. The sun sits at mid elevation
rather than the sim's near-overhead angle because relief is what this view exists to show,
and an overhead sun flattens ruts, rock shadows and hull edges alike.

Recordings carry ABSOLUTE mesh paths from the machine that produced them, so a run copied
off the cluster names `/work1/...` and resolves to nothing locally. Those are re-rooted
automatically: every path has a `data/` segment and what follows it is stable across
checkouts, so the local `data/` and the sibling Chrono checkout are tried in turn.
`--mesh-root DIR` adds more. The line it prints says how many were re-rooted, and anything
still missing is drawn as a box rather than skipped.

An SCM run records no terrain patch -- the deformable terrain is not a static visual when
`ExcludeExisting()` runs -- so the surface is rebuilt from the heightmap named in the
metadata instead.

Deformation is read from `rank_<r>_scm.bin` when the recording carries it, and the ruts are
drawn. Those files are a near-passthrough of `SCMTerrain::GetModifiedNodes()`:

```text
header  8s "AMDUWSCM", u32 version, u32 rank, f64 rate_hz, f64 delta,
        f64 plane[7] (pos xyz + quat wxyz), i32 nx, i32 ny
frame   u32 0x4D435353, f64 time, u32 count, count * (i32 i, i32 j, f32 z)
```

Node `(i,j)` sits at `(i*delta, j*delta)` in the patch plane frame, heights are absolute,
and each frame carries only the nodes modified since the last -- so a consumer accumulates,
and a dropped sample leaves the ground slightly stale rather than permanently wrong.

The ruts get their own fine grid per rank rather than going onto the terrain: SCM nodes are
`delta` apart -- 0.02 m in current runs -- against 4.0157 m per heightmap vertex, so ruts
are two hundred times finer than the ground mesh and cannot be shown on it. Each patch
spans only the bounding box its own rank touched, against the 2.6 billion nodes a 0.02 m
grid over the full 1024 m patch would need.

That box is still enormous. A collector working a 60 x 34 m sector sweeps 3415 x 1809 =
6.2 M nodes, under 9% of which are ever deformed, so the patches are DECIMATED rather than
drawn node for node: `--rut-nodes` (default 2 000 000) is shared out between the ranks and
each patch takes the coarsest stride that fits, which is 0.08 m for a four-rank run. The
line the tool prints names the resulting grid and its spacing. Deformation snaps to the
nearest kept node with the deepest value winning, so a rut keeps the depth it was recorded
with and loses only width resolution; raise `--rut-nodes` for finer ruts at the cost of
memory and frame rate.

Decimating rather than skipping is the point. There used to be a hard node cap, and over it
the patch was dropped -- but the terrain underneath had already been cut away, so the ruts
came out as slabs of background colour with no relief in them. The cut now follows the
patches that actually got built.

The terrain cells beneath each patch are CUT OUT. Biasing one of two coincident surfaces --
polygon offset, a millimetre lift -- only settles the depth-buffer tie, and leaves two lots
of geometry and shading in the same place, which at site distances reads as a rectangular
slab hovering over the ground. Removing the covered cells leaves one surface.

The patch then has to fill the hole EXACTLY, and the two grids make that awkward: terrain
pitch is 4.0157 m (`length/(nv_x-1)`, not a round number) and SCM nodes are 0.02 m apart,
so they are incommensurate and a patch whose edges are node multiples misses the cell
boundary by up to half a node. That leaves a gap showing background down one side of the hole and an
overlap that z-fights down another -- a rectangular outline around the whole driven area.
So the patch carries the cell boundary itself as its outer ring: first and last spacing is
whatever is left over, the interior sits exactly on nodes. The seam is then exact to zero,
because along a shared edge the patch's bilinear base collapses to the same linear
interpolation the coarse quad's edge uses.

Colour is sinkage against the undisturbed surface, with the zero end pinned to the
terrain's own colour so undriven ground is invisible and the eye finds the tracks;
`--scm-depth` sets what counts as fully dark. Cumulative keyframes are handled implicitly:
heights are absolute, so re-applying a full dump is idempotent. Scrubbing backwards rewinds
to pristine and replays, so a scrubbed frame matches the same frame reached by playing
forward. `--no-scm` draws the terrain undeformed.

It is 1:1 with what the sim drew, and that takes more than loading the OBJ files. Every
mesh is fitted to the bounding box the recorder captured, because -- as TrajectoryRecorder
says in as many words -- `scale` alone is a lie for anything transformed in memory after
loading. Two cases here, both badly wrong without the fit: every rock reports scale
[1,1,1] and is drawn at 0.2, since LoadRockMesh bakes the scale into the vertices and
re-bases the mesh on the ground plane, so the source OBJ renders FIVE TIMES too large; and
the builder hull is drawn deliberately squashed in z by 0.110 (`Builder_Chassis_Squashed_Z`
in BuilderRig.cpp, so the roof does not bury the arm), so the source OBJ renders as a
full-height M113. The fit is per axis, which reproduces a one-axis squash exactly where a
uniform scale cannot. Primitives come from the dimensions the recorder wrote -- box `size`,
cylinder `radius`/`height` -- not from a bounding box; run16 carries 92 cylinders.

Scenery comes from `static_props.jsonl` rather than being synthesised. The three rings are
not circles: each is 180 short boxes laid ON the terrain by height probe, following its
height, and the pad and decorative wall rocks are recorded the same way -- a flat circle at
a guessed height is wrong by metres on a hillside. The terrain is rebuilt from the heightmap
named in the recording's own metadata and fitted to the patch bounds, which is what makes
it exact: run16's patch spans z=-13.82..12.65 while its metadata declares [-25, 25], because
Chrono maps grey over the image's own range and not the declared one. Vertex pitch works out
at length/(nv_x-1) = 4.0157 m, matching Chrono's mesh exactly. Checked against the recorded
scenery: ring markers land within a median 0.08 m of the rebuilt surface and the placed
rocks within 0.00 m, the residual being grid pitch on a slope, and builder hulls float
0.36-0.77 m above it, which is the track and road wheels they are actually standing on.

Meshes are loaded once per file and shared, so fifteen ranks of builders cost one hull
mesh rather than fifteen, and each body is drawn in the colour its own shapes carry, so the
playblast looks like the run instead of like a debug view. Bodies stay hidden until their
first recorded pose, because rocks are created during the run and would otherwise sit at
the origin until they exist.

The whole 1024 m patch is drawn at full resolution by default, because the site is not the
whole run: the collectors drive out past 200 m on the harvest lanes, and a terrain cropped
to the rings leaves them flying over nothing. It costs nothing to keep -- the heightmap is
256x256, so the patch is 65k points -- and decimating it flattens exactly the relief the
site cares about. `--terrain-margin <m>` crops to the rings if you want it, and
`--terrain-decimate` coarsens it.

Track shoes and their lugs, road wheels, sprockets, idlers and suspension are drawn by
default: the tracks are most of what a tracked machine looks like, and without them a
builder is a coloured plate sliding over the ground. That is 1905 of the 3336 bodies in a
15-rank run, and `--no-running-gear` takes them back out when frame rate matters more than
looks.

Two things had to be fixed to make that affordable, and both are worth knowing if this
gets extended. Never pass `name=` to `add_mesh` in a loop: pyvista then calls
`remove_actor(name)` first, which scans the whole actor collection by name, so building
3336 bodies was O(n^2) -- 11.2 million VTK collection lookups and three minutes of scene
build. And placed geometry is cached by shape signature, not just by mesh file: 1905 track
shoes are the same mesh fitted to the same box at the same local frame, differing only in
body pose, and body pose lives on the actor. Together those took a full-site frame from
3 min 15 s to 31 s.

### Cost diagnostics

`--perf_log <seconds>` prints a per-rank breakdown every N sim seconds: the
*instantaneous* wall/sim for the window (the plain `wall/sim` line is a cumulative
average, which hides when a cost starts growing — it is why an early slowdown here
read as unbounded growth when it had actually plateaued), the wall time in each
loop section, Chrono's per-step timers, and the builder's drive command / speed /
orbit radius / track-shoe spread.

```bash
mpirun -np 3 ./build/demo_SYN_construction --no_sensor -e 10 --perf_log 0.5
```

`--builder_no_arm` drops the manipulator, to separate arm cost from vehicle cost.

Rank layout:

```text
rank 0 = global sensor/visualization rank
rank 1 = robot 1   (collector 1 + builder 1)
rank 2 = robot 2   (collector 2 + builder 2)
rank 3 = robot 3   (collector 3 + builder 3)
rank 4 = robot 4   (collector 4 + builder 4)
rank 5 = robot 5   (collector 5 + builder 5)
```

Terminal 4, optional status/debug commands:

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

### End of mission: return home and dump

Once every rock is either collected or skipped, the collector drives back to its drop
point, stops, tips its bed to drop the load, resets, and stays put. The chain is:

```text
manipulator_controller  all targets resolved   -> /robot_N/mission_done
pure_pursuit_controller drives to /robot_N/homePos, stops
                                               -> /robot_N/at_home
manipulator_controller                         -> /robot_N/trailer_cmd [1]
C++ RobotRig            runs the dump cycle    -> /robot_N/trailer_state
```

`/robot_N/trailer_state` is `[state, bed_angle_rad, tailgate_angle_rad]`, where
state is `0` idle, `1` opening gate, `2` tilting, `3` dwell, `4` levelling,
`5` closing gate, `6` done. `/robot_N/homePos` is published by the C++ drive
bridge so the drop point is not recomputed in Python.

**The last leg is tangential, and it stays outside the ring.** The rock line runs
radially *outward* from the drop point, so a rover left to itself comes home radially —
nose-in at the circumference, trailer pointing out into open field, load tipped wherever
it happened to stop.

The first fix for that followed the collector circle itself: an entry waypoint on the
circle 12 m of arc *clockwise* of the drop point, then the drop point, reached by
following the circle round. It arrived tangentially and it also drove into builders.
Clockwise is where the builder *is* — it walks counter-clockwise towards the drop point —
so the last 12 m of every return leg ran down the builder's own lane with 4 m of radius
between them (37 against 33, hull half-width 1.343 m), and `drop_band_half_width_m` of
2 m let an arrival at r=35 count, 0.66 m off the tracks.

The run-in is now an **arc that lies outside the collector circle and touches it only at
the drop point**. Put its centre on the drop point's ray at radius `ring + rho`, with
radius `rho` (`approach_arc_radius_m`, 12 m): the closest that circle comes to the site
centre is `(ring + rho) - rho = ring`, at the drop point, and every other point on it is
further out. Three properties fall out of that one construction:

- the rover can never be carried inboard of the ring, so the builder's orbit is
  unreachable from the return leg — measured minimum path-to-hull-centre is 4.14 m,
  against a 1.343 m hull half-width;
- the arc is tangent to the ring at the drop point, so the heading there is tangential and
  the rear-discharging trailer pours *along* the circumference;
- travel is **clockwise** about the site, so the rover descends from the counter-clockwise
  side — the side the builder is never on — and its trailer points the way the builder
  walks, laying the load ahead of the machine rather than behind it.

`rho` is the only knob and must clear the rover's ~5 m turn radius with margin; 12 m is
2.4x it, and the entry point sits at r=50.4 m, well clear of everything.

**The drop point follows the builder, not the cycle counter.** `/builder_N/arm_status`
carries three appended fields — the slot being consumed, how many unlaid rocks the builder
still owns, and that slot's angle about the site centre — and the collector places its load
`drop_slots_ahead` (1) slots past it, keeping the radius the sim chose. Reach is what bounds
that number: arm base to pile is 3.91 m at 0 slots, 4.04 at 1, 4.43 at 2 and 5.01 at 3,
against `feedstock_reach_max` = 5.0 m, so three slots ahead is refused outright. If the
builder goes quiet the drop point falls back to the sim's own, so a lost topic degrades
instead of stranding a loaded rover.

That also let `drop_arc_tolerance_m` come down from 8 m to 3 m. The 8 m existed because
a rover driving straight at the drop point cannot converge on it — pure pursuit orbits a
target inside its own turning radius — so arrival had to be accepted from a long way
out. With the run-in, that much slack would be actively harmful: the outboard arc puts the
rover on the ring only at the drop point, so wide arc slack buys nothing on the way in and
simply parks it short of the pile it is meant to be building.
Arrival is also accepted on a **stop line** — committed to the run-in and past the drop
point in arc — because "have I passed it" cannot be missed by a rover carrying a little
too much speed, whereas "am I near it" can be sailed through in one control period.

The cycle runs in C++ at the simulation step rate, and it has to. Both the bed and
the tailgate are `ChLinkMotorRotationAngle`, so re-setting the commanded angle is
a *position* discontinuity — an instantaneous velocity that throws the rocks out
of the tub instead of tipping them out. The cycle therefore slews each angle at a
bounded rate (bed 12 deg/s to 55 deg, gate 60 deg/s to 95 deg, 3 s dwell at full
tilt), which cannot be driven from a 10 Hz controller. A controller only asks for
a cycle; repeat requests during one are ignored.

55° and not 40°: the load has to *slide*, and a rock on the bed slides only when
tan θ > μ. The bed is μ = 0.9, so the threshold is arctan(0.9) = 42.0°. At 40° the
load is below it and is not supposed to move at all — the dumps that appeared to work
were rocks rolling or being nudged by the tilt, which is why three ranks emptied and
the fourth kept its rock sitting 0.28 m from the bed centre. Gravity cancels out of
tan θ > μ entirely, so lunar gravity never entered into this and it failed exactly as
it would on Earth.

The tub is **centred on the trailer and discharges rearward**, over the -x lip, with
the tilt axis *on* that lip so the pour line stays put instead of walking forward under
the trailer as the tub rises. It spent a while discharging to the left instead, which
forced the tub off-centre — the left tire spans y = 0.35–0.65 with its top only 21 mm
below the bed lip, so a symmetric tub pouring sideways empties onto its own wheel.
Nothing is behind the tailgate but ground, so rear discharge has no such conflict, and
the tub sits where `RosArmBridge`'s 4×4 placement grid has always aimed.

The gate opens before the bed tilts and closes after it levels, so the load is
never pressed against a closed gate and then released all at once.

To drive a cycle by hand:

```bash
ros2 topic pub --once /robot_1/trailer_cmd std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic echo /robot_1/trailer_state
```

At full tilt the sim logs both bed lip heights, e.g. `front_lip_z=6.010
rear_lip_z=5.355 drop=0.655`. That check exists because getting the motor axis
sign wrong does not fail loudly — it would press the load against the closed
front wall and dump nothing at all. A positive drop means the open rear is lower,
which is correct.

### Stray controllers: check this first

`ros2 launch` spawns its nodes as children and does **not** take them down when
the launch process itself is killed. Ctrl-C in the terminal is fine, but killing
the launch PID (or a script doing `kill $!`) leaves a full set of controllers
running. They are invisible until the next run, when they fight it:

```bash
# Before every run:
pgrep -af "pure_pursuit_controller|manipulator_controller"   # expect nothing
# Clean up:
pkill -9 -f "pure_pursuit_controller|manipulator_controller"
# From a script, launch in its own process group and kill the group:
setsid ros2 launch amd_uw_ros2 robot_controllers.launch.py ... &
kill -TERM -$!
```

Two duplicate controllers produce two distinct failures that look like anything
but duplication:

- **On `vehicle_cmd`**: commands alternate, so braking flickers between 0 and 1,
  throttle never sticks, and the rover sits at spawn. Reads as a broken vehicle.
- **On `arm_cmd`**: each manipulator node caches its own `arm_base_pose` and
  solves its own IK, so one rock produces several different answers. The arm
  grabs at the wrong place, or calls a rock it is parked beside unreachable. A
  stale node also still holds rock positions from an *earlier* sim.

Both are now reported by the sim on the first offending step, naming the `pkill`.
If you see `N publishers on ... -- expected 1`, stop and clear strays; results
until then are not trustworthy.

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
4. [x] Move to SCM terrain — 1024 x 1024 m at 0.10 m grid spacing.
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
