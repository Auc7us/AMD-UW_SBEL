# AMD-UW SynChrono Demo — design notes

Why the site is laid out and tuned the way it is, and what the recording formats contain.
Running instructions are in [README.md](README.md). Everything here is a conclusion that
cost a run to reach; the numbers are measured, not chosen.

## Site layout

Concentric about `(0, 0)`. Each rank owns one evenly spaced ray, `2*pi*rank/N`:

```text
centre -- work circle (50 m) -- builder orbit (53 m) -- collector ring (57 m) --> rock line
             yellow                    cyan                    green
```

`src/RobotLayout.h` is the single source of truth for all of it. It also scales past two
ranks; an earlier layout hard-coded two headings and silently gave every rank after the
second the same heading as rank 2.

**The radial gaps are set by machine dimensions, not by site size.** Scaling the site
changes the radii and leaves the gaps alone:

| gap | size | set by |
|---|---|---|
| wall → builder lane | 3.0 m | the arm's reach band (2.78–4.44 m proved in service); one lane must serve both the wall and the drop pile |
| lane → collector ring | 4.0 m | clearance between a 2.686 m hull and a 1.49 m rover |
| ring → rover spawn (62 m) | 5.0 m | staging, so a spawning trailer does not land on its own builder |

5 m gaps both sides were tried and the builder had to reposition for every pick.

What growing the circle buys is **arc per rank**, which is what lets builders rotate as
they build: at 16 ranks the wall sector goes from 11.78 m to 19.63 m, and the course from
one arm-station to nearly two.

Collectors start directly outside their own builder, face radially outward, and their rock
line runs away from the site — so no rock line crosses the build area. Builders sit tangent
to their orbit, so their radial footprint is only the hull width.

The rock line runs 30–660 m out and its outer end leaves the 1024 m heightmap. Nothing
needs stitching: SCM's patch extends to infinity in x-y and an uninitialized node takes the
height of the nearest initialized one, so a rock past the edge rests on flat ground at the
right elevation.

**Terrain constraint.** The levelled pad is `make_graded_pad.py --pad-radius`, and the site
now reaches r = 62 m at the spawn ring. Height spread measured 0.11 m rms inside r = 20 m
rising to 0.29 m at 62–65 m. Regrade before moving the rings outward again.

### Derived geometry

Everything comes from the slot index `k`, so arm, drive station and feedstock cannot
disagree:

| | radius | angle |
|---|---|---|
| wall slot `k` | 50.0 | `ray + k·pitch` |
| station for `k` | 53.0 | `ray + k·pitch + arm_lead` |
| seed heap | 56.6 | heap-centre slot angle, **no** `arm_lead` |
| collector load `c` | 57.0 | `ray + HarvestDropSlot(c)·pitch` |

`pitch = wall_slot_pitch_m / work_circle_radius`. `arm_lead = atan(2.5/53)` exists because
the arm mounts 2.5 m *back* along a tangentially parked hull; adding it to the station puts
the **arm base**, not the hull origin, radially opposite the slot it serves.

That is why the heap must **not** also carry it: doing so pushed every heap to 5.62 m from
the arm base, past the 5.2 m guard, and no rock was reachable. `main.cpp` prints a reach
audit at startup so that class of error shows up at t=0 instead of after the machine has
driven to its first station.

## The station bands must be fractions of the slot pitch

This one cost a 5-hour 8-rank run. `station_tolerance_rad` is an **angle** and arrival is
judged with it, so its arc grows with the lane radius:

| | working config | after scaling the site |
|---|---|---|
| lane radius | 33 m | 53 m |
| `station_tolerance_rad` | 0.015 | 0.015 (left alone) |
| **arrival tolerance in metres** | **0.495 m** | **0.795 m** |
| slot pitch | 0.9 m | 0.5 m |
| tolerance ÷ pitch | 0.55 | **1.59** |

Above 1.0, every newly published station is already inside the arrival band, so
`holding_station` latches immediately, the builder **never enters the drive branch**, and
the forward creep to the next slot — the only thing that corrects radial error — never runs.
Measured on `run_20260827_061917`: all eight builders spiralled 6–7 m inside the lane, took
station off-lane, starved, and froze. Twenty rocks in 600 s.

The mechanism that corrects radius is the ordinary forward creep: pure pursuit aims at a
point **on** the lane, so every advance pulls the radius in. Correcting in place does not
work and was tried — pivoting a braked tracked hull on regolith walks it further off the
lane than the creep pulls it back, measured going `-0.40 → -1.30 → -2.10 → -2.86 → -3.38 →
-3.75 → -3.95 m` over six attempts. Reverted.

**How the drift starts.** At `v/ω = 2.770 m` against a 2.70 m hull half-length, the builder
was rotating about one end of itself — a skid-steer pivot, not a chosen turn. It pivots
because the station-keep steering cap barely clears the standing trim the lane needs:

```
standing trim at R = 53:  |(1/53 - hull_bias 0.0952) / gain 0.314|  =  0.243
old fixed cap                                                      =  0.350
headroom for correction                                            =  0.107   (31%)
```

The pursuit law asks for ≈0.48 when ~1 m off-lane, the cap chops it to 0.35, steering sits
pinned from the first tick, and pinned steering at 0.08 m/s is a pivot. Once the hull is
rotating, every metre forward goes inward, which asks for more steering. Self-sustaining.

So all three are **derived**, never hardcoded:

```python
station_tolerance_rad()   = 0.55 × slot_pitch_rad          # must stay < 1.0 × pitch
station_keep_deadband_m() = 0.28 × slot_pitch_rad × radius
station_keep_steer_cap()  = min(steering_limit, |steering_ff| + 0.15)
```

Those fractions reproduce the hand-set values that worked (0.495 m, 0.252 m, 0.357) to
within 1% at the old geometry, and follow the site when it moves.

The slot pitch itself is **measured off the station stream** — a running minimum of the
forward step between consecutive stations, so a skipped slot cannot widen the bands.
`wall_slot_pitch_m` is only a bootstrap for the first station. Worth it: the measured pitch
is **0.530 m**, not the 0.500 m bootstrap, because slots lie on the 50 m wall while the
builder rides the 53 m lane. Hardcoding the angle would have been 6% tight.

The controller logs the derived bands at startup and errors if the arrival band ever
reaches a whole pitch again.

## What the builder does

The builder builds at its own pace; the only thing it takes from its collector is rock.
One wall slot per iteration:

1. Sim publishes `/builder_N/pick_target` with `ready=1` — hull parked, slot `k` unlaid, a
   rock inside the arm envelope.
2. `builder_arm_controller` solves grab and place IK in the arm base frame, sends one
   12-element `/builder_N/arm_cmd`.
3. `LrvArm`'s pick/place machine runs it: approach, close on contact, lock, lift, swing
   180°, settle, release, stow.
4. Sim re-fixes the laid rock and advances the station angle by one slot.
5. `builder_orbit_controller` creeps one pitch and parks. `ready` goes high for `k+1`.

**A station may only ever move forward.** Releasing it for any other reason hands the
builder a lap, and at sixteen ranks a lap drives one builder through the next one's station.
`SetStationAngle` clamps monotonically and reports a refused retreat once per episode.

### The hull is held by a force, not a constraint

While parked the hull is held by a spring-damper **force** on the chassis (5e4 N/m,
4.5e4 N·s/m plus a yaw pair, horizontal only — vertical is the suspension's job). Full brake
with zero throttle is the park signal, which is what the orbit controller already publishes
on station, so no extra topic is needed. A braked M113 does not stay put: measured, it
creeps at 0.22–0.27 m/s on full brake, so `m_parked` never went true and the arm was never
offered a pick.

It must not be `SetFixed(true)`. That does hold the hull and it **destroys the track**: the
body leaves the solve, its velocity goes to zero in one step while ~130 shoes are still
solving against last step's velocity. Bisected at 3 ranks with no controllers — pinning on
brake died at t=2.115, pinning only below 0.02 m/s died at t=6.085 (the velocity gate was a
hypothesis and it was wrong; it only delayed the failure), and disabling the park entirely
ran clean past t=21. (That bisection flag is gone; the finding is recorded in
`BuilderRig.cpp`.) A force has no velocity discontinuity.

Seating the hull on a *fitted* terrain plane (`SeatBuilderOnTerrainPlane`, worst-case error
0.02–0.08 m against 0.24–0.33 m for single-probe placement) is still needed and orthogonal:
that is about not injecting strain at t=0.

### The kingpin singularity

The tie rod is a `ChLinkDistance`, a two-solution constraint. The kingpin axis runs
UCA_U(-0.008, 0.571, 0.088) → LCA_U(0.016, 0.574, -0.082), 8.1° off vertical; the tie rod is
0.44548 m and the steering arm 0.1199 m, so `|P_c − P_u| = 0.44548` is satisfied at θ = 0
**and at θ = −108°**. A wheel that reaches the far branch stays there — observed locked at
−117.1°, which reads as a snapped axle.

`ApplySteeringStops` therefore makes the singularity unreachable: a spring-damper torque
accumulator on each upright past `stop_angle = 0.75` rad, and an alarm at 1.20 rad.

Each stop must bind to **its own** upright. Body names are not unique —
`TrajectoryRecorder.cpp:295` invents `#N` suffixes — so a name lookup bound both axles to
the *front* upright, leaving the rear unguarded and the front double-torqued. Match by
proximity to the axle's own spindle instead.

## Feedstock accounting

**Rocks per rank is 2–6, drawn per rank**, not a site-wide constant.
`RocksPerRank(rank_index)` is a pure integer hash of the rank index rather than anything
from `<random>`, because three places in *different processes* must arrive at the same
number: `RockField` spawns that many bodies; `SynRockAgent` sizes the sensor rank's zombie
pool for a rank it knows nothing about but the id (undersize it and that rank's rocks are
invisible in the global camera); and `HarvestDropSlot` advances the collector's lane by one
load's worth of wall slots. `--rocks_per_rank N` pins it for reproducible tests.

That third consumer is why a bigger load is not just "more rocks". A load is tipped in one
place and the builder creeps one slot per rock, so with `L` rocks the far end of the run is
`L-1` slots away. At `L=6` the reach audit measured 6.54 m against a 5.2 m envelope. Loads
are therefore dropped at the **middle** of the run of slots they serve
(`HarvestLoadCenterSlot`), exactly as the seed heap is centred on its own run: 4.70 m at
`L=6`, and it tightens `L=2` from 4.04 to 3.94 m.

**One seed heap of six rocks per builder is all the site places.** It exists to give the
builder something to lay during its collector's first outbound leg, which is minutes of sim.
Everything after is delivered, and it works because the harvest lane advances in **wall
slots**, not degrees: load `c` lands at `HarvestDropSlot(c) = 6 + 2c`, exactly where the wall
will have reached. The step used to be a flat 30° — 17 slots at the old pitch — which left a
wall 88% gaps and every pile 15 m of arc from the builder meant to eat it.

The arm bridge does not index its feedstock. It takes the **nearest un-consumed rock inside
the envelope**, seed heap and delivered loads pooled. That has to be a search: a load is
tipped from a moving trailer and lands where it lands. One rule for both sources also means
the heap-to-delivery changeover needs no handling. Failed grabs retry up to
`max_grab_attempts = 3` without skipping the slot; measured 6 retries → 1 write-off.

**Every rock is fixed except the one in the gripper.** Seed rocks are created
`SetFixed(true)` with collision on; delivered rocks freeze the same way once at rest.
`LrvArm::TryLockRock` unfixes exactly one on finger contact and the bridge re-fixes it once
laid — it is part of the wall then and must not be nudged by the next stone landing beside
it. Delivered rocks used to be *released* when a builder came within 3 m or pushed on them;
both triggers fire exactly when the arm is trying to take hold.
`UpdateRockCollisionActivation` also skips fixed rocks, or the distance test switches
collision off on the pile the gripper is closing on.

Feedstock the builder has driven past is deleted (`ClearStrandedFeedstock`, 15° behind with
a 30 s grace) rather than tracked back for. Tracking back is what a 16-rank site cannot
afford.

## One ChSystem per rank

Each physics rank has exactly **one** `ChSystem` holding that rank's rover, trailer, rocks,
terrain and builder. Hard requirement: Chrono only generates contacts between bodies of the
same system, so a builder in its own system could never touch the rocks it exists to place.

The whole system therefore runs the solver the single-pin track needs — `ChSolverBB` at 100
iterations with `EULER_IMPLICIT_LINEARIZED`, set once in `main.cpp`; the default solver lets
shoes drift off the road wheels. `BuilderRig` deliberately does not touch gravity, solver,
timestepper or terrain: it does not own that world.

Two ordering rules, both silent when broken:

- Every `Synchronize` runs before any `Advance`, and the system steps exactly once per loop
  iteration. `BuilderRig::Advance` advances only its own subsystems; `robot->Advance` issues
  the `DoStepDynamics`.
- The builder is constructed **after** the rover's SolidWorks arm import, which leaves the
  collision system in a state where later bodies never register contacts. `BuilderRig` calls
  `BindAll()` to repair it — without it the tracks sink through the ground and the gripper
  passes through rocks.

## Terrain: deformable SCM

`SCMTerrain` (Bekker-Wong) over the 1024 × 1024 m `terrain2_graded.png` heightmap. Soil
parameters and resolution are the `terrain_scm_*` / `scm_*` constants at the top of
`main.cpp`. Read the cost before changing `--scm_delta`:

| | at 0.10 m |
|---|---|
| grid nodes | 10241² = **104.9 M** |
| undeformed heights (dense `ChMatrixDynamic<double>`) | **839 MB per rank** |
| visualization mesh (verts + normals + UV + colour + 209.7 M faces) | **~13.5 GB per rank** |

Both scale as `1/delta²`, and the height matrix is allocated whether or not a node is ever
touched. The visualization mesh is therefore built **only on ranks that render** — a rank
with its own `--vsg` window, or rank 0 with its camera on. Physics is unaffected: SCM works
from the height matrix and the active domains, not from the mesh.

**Active domains are not optional.** SCM only casts rays from nodes inside some active
domain, so a body outside every domain gets no soil reaction and sinks through the terrain.
Four places register them and all four are needed:

- `RobotRig::InitializeOnTerrain` — collector and trailer wheel spindles, cycle-0 rocks.
  Before `Settle()`, or the settle steps run against the whole grid.
- `RobotRig::StartNextHarvestCycle` — each later cycle's rocks as they spawn.
- `main.cpp` after the builder is built — **one** domain on the M113 hull, not ~130 on the
  shoes (a domain only selects which nodes cast rays; the rays hit whatever shape is above
  them, so one box over the footprint covers every shoe), plus one per seed rock.
- `main.cpp` on ranks owning no robot — a degenerate 1 cm domain on a marker body. Rank 0
  steps the system too, and `SCMLoader::SetupInitial()` only installs Chrono's default
  whole-system domain when a visualization mesh was requested, so a headless rank 0 would
  index an empty domain list behind a release-disabled assert.

For reference, the rigid terrain this replaced was a ~130k-triangle collision mesh queried
every step by ~130 track shoes: ~7.6 of ~17.9 wall/sim. SCM moves that cost from Bullet's
narrowphase to ray casting over the active domains, so it scales with how much machine is
on the ground, not with map size.

## Recording formats

Recording is **on by default** — a run nobody can re-render is a run that has to be
repeated, and at this terrain size a repeat is expensive. Rank 0 stamps the timestamped
directory name once and broadcasts it over MPI; every rank deriving it from its own clock
would scatter one run across as many directories as there are ranks the moment the second
ticked over during startup, and startup here is seconds of heightmap resampling.

Poses, `rank_<r>_frames.bin`, little-endian:

```text
header  8s "AMDUWTRJ", u32 version, u32 rank, f64 rate_hz, f64 step
frame   u32 0x544A5246, f64 time, u32 count,
        count * (u32 index, f32 px,py,pz, f32 qw,qx,qy,qz)
```

The pose is the body's **reference** frame, which is what Chrono hands its renderers, so a
visual shape's world transform is `body_pose * shape["pos"], shape["rot"]` from
`rank_<r>_objects.jsonl`. A body is not necessarily one mesh at its own origin.

Deformation, `rank_<r>_scm.bin`, a near-passthrough of `SCMTerrain::GetModifiedNodes()`:

```text
header  8s "AMDUWSCM", u32 version, u32 rank, f64 rate_hz, f64 delta,
        f64 plane[7] (pos xyz + quat wxyz), i32 nx, i32 ny
frame   u32 0x4D435353, f64 time, u32 count, count * (i32 i, i32 j, f32 z)
```

Node `(i,j)` is at `(i*delta, j*delta)` in the patch plane frame, heights are absolute, and
each frame carries only nodes modified since the last — so a consumer accumulates, and a
dropped sample leaves the ground slightly stale rather than permanently wrong. Absolute
heights also make re-applying a full dump idempotent.

An SCM run records no terrain patch — deformable terrain is not a static visual when
`ExcludeExisting()` runs — so consumers rebuild the surface from the heightmap named in the
metadata.

### Verify a consumer before writing one

`read_trajectory.py --bbox 0` rebuilds one frame from the files alone
(`body_pose * shape_frame * shape_aabb`) and prints each group's world extent. Getting a
convention wrong does not error, it just produces a wrong scene. A correct rebuild reads:

```text
builder    size=  2.705 x   5.407 x  1.146 m     <- M113 hull is 2.686 m wide, 5.4 m long
collector  size=  3.511 x   2.496 x  2.070 m
rock       size= 49.723 x  18.025 x  1.863 m     <- the group, i.e. the whole rock line
```

Wheels folded into the hull (the centre-of-mass mistake) collapses the collector's extent;
ignoring shape frames stacks the M113's road wheels on its centreline.

**`scale` alone is a lie** for anything transformed in memory after loading, so every mesh
must be fitted to the bounding box the recorder captured. Two cases here are badly wrong
without the fit: every rock reports scale `[1,1,1]` and is drawn at 0.2, because
`LoadRockMesh` bakes the scale into the vertices and re-bases the mesh on the ground plane —
the source OBJ renders five times too large; and the builder hull is deliberately squashed
in z by 0.110 (`Builder_Chassis_Squashed_Z`, so the roof does not bury the arm), so the
source OBJ renders as a full-height M113. The fit must be **per axis** to reproduce a
one-axis squash. Primitives come from recorded dimensions (box `size`, cylinder
`radius`/`height`), not from a bounding box.

Recordings carry **absolute** mesh paths from the machine that produced them, so a run
copied off the cluster names `/work1/...`. Every path has a `data/` segment and what follows
it is stable across checkouts, so re-root on that.

Scenery comes from `static_props.jsonl` rather than being synthesised. The three rings are
not circles: each is 180 short boxes laid **on** the terrain by height probe. A flat circle
at a guessed height is wrong by metres on a hillside. The terrain rebuild is exact when
fitted to the recorded patch bounds — `run16`'s patch spans z = -13.82..12.65 while its
metadata declares `[-25, 25]`, because Chrono maps grey over the image's own range, not the
declared one. Vertex pitch is `length/(nv_x-1) = 4.0157 m`. Checked: ring markers land
within a median 0.08 m of the rebuilt surface, placed rocks within 0.00 m, and builder hulls
float 0.36–0.77 m above it — the track and road wheels they stand on.

### Drawing ruts

Ruts need their own fine grid per rank rather than going onto the terrain: SCM nodes are
`delta` apart against 4.0157 m per heightmap vertex, so ruts are two hundred times finer
than the ground mesh. Each patch spans only its own rank's bounding box — a collector
working a 60 × 34 m sector sweeps 6.2 M nodes, under 9% ever deformed — so patches are
**decimated**, not drawn node for node. Deformation snaps to the nearest kept node with the
deepest value winning, so a rut keeps its depth and loses only width resolution. There used
to be a hard node cap that *dropped* the patch over it, but the terrain underneath had
already been cut away, so the ruts came out as slabs of background colour.

The terrain cells beneath each patch are **cut out**. Biasing one of two coincident surfaces
(polygon offset, a millimetre lift) only settles the depth-buffer tie and leaves two lots of
geometry in the same place, which at site distances reads as a slab hovering over the
ground.

The patch then has to fill the hole exactly, and the grids are incommensurate: terrain pitch
4.0157 m against 0.02 m nodes, so a patch whose edges are node multiples misses the cell
boundary by up to half a node — a gap down one side and a z-fighting overlap down the other,
i.e. a rectangular outline around the driven area. So the patch carries the cell boundary
itself as its outer ring: first and last spacing is the leftover, the interior sits exactly
on nodes. The seam is then exact, because along a shared edge the patch's bilinear base
collapses to the same linear interpolation the coarse quad's edge uses.

### Two performance facts for anyone extending replay_run

Never pass `name=` to `add_mesh` in a loop: pyvista calls `remove_actor(name)` first, which
scans the whole actor collection, so 3336 bodies was O(n²) — 11.2 M VTK lookups and three
minutes of scene build. And cache placed geometry by **shape signature**, not just mesh
file: 1905 track shoes are the same mesh fitted to the same box at the same local frame,
differing only in body pose, and pose lives on the actor. Together: 3 min 15 s → 31 s for a
full-site frame.

Pushing poses dominates rendering — ~88 ms a frame against ~12 ms to draw — so a four-rank
run with running gear has a ~10 fps ceiling. `--max-frames` (default 3000) caps frames held
in **memory**, ~22 kB per frame for a 15-rank run; over it the recording is resampled and
plays coarser than recorded.

## End of mission: return home and dump

Once every rock is collected or skipped, the collector drives back, stops, tips its bed,
resets and stays put:

```text
manipulator_controller   all targets resolved  -> /robot_N/mission_done
pure_pursuit_controller  drives to /robot_N/homePos, stops
                                               -> /robot_N/at_home
manipulator_controller                         -> /robot_N/trailer_cmd [1]
C++ RobotRig             runs the dump cycle   -> /robot_N/trailer_state
```

`homePos` is published by the C++ drive bridge so the drop point is not recomputed in
Python.

**The last leg is an arc outside the collector circle, touching it only at the drop point.**
The rock line runs radially outward, so a rover left to itself comes home nose-in with its
trailer pointing into open field. Following the ring instead was tried and drove into
builders: 12 m of arc *clockwise* of the drop point is exactly where the builder is, so the
last 12 m ran down its lane with 4 m of radius between them.

Put the arc's centre on the drop point's ray at radius `ring + rho`, with radius `rho`
(`approach_arc_radius_m`, 12 m). The closest that circle comes to the site centre is
`(ring + rho) - rho = ring`, at the drop point; every other point is further out. Three
properties fall out of that one construction:

- the rover can never be carried inboard of the ring, so the builder's orbit is unreachable
  from the return leg — measured minimum path-to-hull-centre 4.14 m against a 1.343 m hull
  half-width;
- the arc is tangent at the drop point, so heading there is tangential and the
  rear-discharging trailer pours *along* the circumference;
- travel is clockwise about the site, so the rover descends from the side the builder is
  never on, and its trailer points the way the builder walks — laying the load ahead of the
  machine rather than behind it.

`rho` must clear the rover's ~5 m turn radius with margin; 12 m is 2.4×.

**The drop point follows the builder, not the cycle counter.** `/builder_N/arm_status`
carries the slot being consumed, how many unlaid rocks the builder still owns, and that
slot's angle, and the collector places its load `drop_slots_ahead` (1) past it. Reach bounds
that number: arm base to pile is 3.91 m at 0 slots, 4.04 at 1, 4.43 at 2, 5.01 at 3 against
`feedstock_reach_max = 5.0`, so three slots ahead is refused outright. If the builder goes
quiet the drop point falls back to the sim's own, so a lost topic degrades instead of
stranding a loaded rover.

Arrival is accepted on a **stop line** — committed to the run-in and past the drop point in
arc — because "have I passed it" cannot be missed by a rover carrying too much speed,
whereas "am I near it" can be sailed through in one control period.

### The dump cycle runs in C++, at the step rate

Both bed and tailgate are `ChLinkMotorRotationAngle`, so re-setting the commanded angle is a
**position** discontinuity — an instantaneous velocity that throws the rocks out instead of
tipping them out. The cycle slews each angle at a bounded rate (bed 12 °/s to 55°, gate
60 °/s to 95°, 3 s dwell at full tilt), which cannot be driven from a 10 Hz controller. A
controller only asks for a cycle; repeat requests during one are ignored.

**55° and not 40°**: the load has to *slide*, and a rock slides only when `tan θ > μ`. The
bed is μ = 0.9, so the threshold is `arctan(0.9) = 42.0°`. At 40° the load is below it and is
not supposed to move at all — the dumps that appeared to work were rocks rolling or being
nudged, which is why three ranks emptied and the fourth kept its rock 0.28 m from the bed
centre. Gravity cancels out of `tan θ > μ` entirely, so lunar gravity never entered into it.

The tub is **centred on the trailer and discharges rearward**, over the -x lip, with the
tilt axis *on* that lip so the pour line stays put instead of walking forward as the tub
rises. Discharging to the left forced the tub off-centre — the left tire spans y = 0.35–0.65
with its top only 21 mm below the bed lip, so a symmetric tub pouring sideways empties onto
its own wheel. The gate opens before the bed tilts and closes after it levels, so the load is
never pressed against a closed gate and then released all at once.

At full tilt the sim logs both lip heights, e.g. `front_lip_z=6.010 rear_lip_z=5.355
drop=0.655`. Getting the motor axis sign wrong does not fail loudly — it would press the load
against the closed front wall and dump nothing. A positive drop means the open rear is lower.

## Duplicate controllers: what they look like

Two controllers on one topic produce failures that look like anything but duplication. The
check and the cleanup are in README.md; this is what you are looking at if you miss it:

- **On `vehicle_cmd`** — commands alternate, braking flickers between 0 and 1, throttle
  never sticks, and the rover sits at spawn. Reads as a broken vehicle.
- **On `arm_cmd`** — each manipulator node caches its own `arm_base_pose` and solves its own
  IK, so one rock produces several different answers. The arm grabs in the wrong place, or
  calls a rock it is parked beside unreachable. A stale node also still holds rock positions
  from an *earlier* sim.

## Cross-rank coordination goes over ROS 2, not SynChrono

`draws_zombies` is `is_sensor_rank && !no_sensor` and every run passes `--no_sensor`, so no
rank ever builds a zombie body for another rank's builder and the remote agents are
`SynQuietAgent`s that drop their messages. Anything one rank needs to know about another's
machines has to come over ROS 2, where every builder already publishes
`/builder_N/vehicle_state` on a shared DDS domain. The conveyor hold that keeps builders
from closing on each other is built on that.
