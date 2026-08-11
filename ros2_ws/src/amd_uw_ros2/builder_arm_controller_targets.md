# Builder build cycle

Working note for `builder_arm_controller.py`, `BuilderArmRosBridge` and the build-plan
block in `RobotLayout.h`. Kept because the geometry is not obvious from the code and
getting it wrong has twice produced an arm that waved at nothing.

## What the builder does

It builds. On its own. The loop is one wall slot long:

1. The sim publishes `pick_target` with `ready=1` — meaning the hull is **parked**, slot
   `k` is unlaid, and a rock is in the arm's envelope.
2. This node solves grab and place IK **in the arm base frame** and sends one
   12-element `arm_cmd`.
3. `LrvArm`'s pick/place state machine runs it: approach, close on contact, lock, lift,
   swing, settle, release, stow.
4. The bridge re-fixes the laid rock and increments the placed count.
5. `main.cpp` turns that count into the next station angle. The orbit controller creeps
   ~1 m. Then `ready` goes high again for slot `k+1`.

Nothing in that loop is *triggered* by the collector's harvest cycle, its dump, or its
mission state — the only thing the builder takes from its collector is rock. Previously
`SetStationAngle` was `RankRayAngleRad(..., robot->GetHarvestCycle())`, so the builder sat
still for a whole harvest and then jumped 30° in one step.

## Why one rank laid a quarter of what the others did

Measured, not guessed. Rank 3 (terrain tilt 5.2°, the second steepest of four):

    holding station at 186.1 deg.
    -0.90 m off the 33.0 m lane; driving the lane back before taking station.
    pushed off station; driving round to re-acquire.

Twice in 10 s of sim, both times at exactly −0.90 m — chattering on a single 0.9 m
threshold. It was not slow at picking, driving or solving. It kept being **sent away**,
and each re-acquisition is a lap's worth of lane at 0.9 m/s.

Two compounding faults, one each side:

- **Sim side.** The virtual anchor latched the raw parked pose, so whatever radial error
  the builder arrived with was held, and on sloped regolith it grew every time it parked.
  The anchor now slews its setpoint radially toward `builder_path_radius` at 0.15 m/s,
  capped at 0.15 m of spring extension (7.5 kN on a 10.5 t hull, ~0.7 m/s²) so it walks
  the hull back rather than yanking it.
- **Controller side.** One threshold did both jobs. Dropping station keeping also releases
  the anchor — so the correction the builder needs is exactly what leaving station switches
  off. There are now two bands: `station_radius_tol_m = 0.9` to take a station,
  `station_radius_release_m = 1.05` to give one up.

The gate could not simply be widened. At station the wall slot is 3.095 m from the arm
base and a delivered load ~3.95 m, so against the `[2.0, 5.2]` envelope the usable radial
band is `[−1.1, +1.95]` m. 0.9 was already most of the inboard side of it.

## The geometry, all from one integer

`k` is the wall slot index. `ray` is the rank's site ray. `arm_lead = atan(2.5/33) =
0.0757 rad`.

| | radius | angle | distance from arm base |
|---|---|---|---|
| wall slot `k` | 30.0 | `ray + k*pitch` | 3.095 m |
| **station** for `k` | 33.0 | `ray + k*pitch + arm_lead` | — |
| arm base when on station | 33.095 | `ray + k*pitch` | — |
| seed heap (one, slots 0–5) | 36.6 | heap-centre slot angle | 3.54–4.37 m |
| collector load `c` | 37.0 | `ray + HarvestDropSlot(c)*pitch` | 3.91–4.04 m |

`pitch = 0.9 m / 30 m = 0.03 rad`, so one slot is **0.9 m of course and 0.99 m of lane**.
Those last two columns are the startup reach audit's own numbers, at 4 ranks.

`arm_lead` is why the table works: the arm mounts 2.5 m **back** along a hull parked
tangentially, so adding it to the station puts the *arm base*, not the hull origin,
radially opposite the slot it is serving. Both reaches then land mid-band automatically.

Arm `+Y` is radially **outward** (`arm_rot = chassis_rot * QuatFromAngleZ(CH_PI)` in
`BuilderRig`), so pick and place are 180° apart in `theta1` — measured `+1.571` /
`-1.571`. An early version swung 0.55 rad between hand-picked angles, which is why the
gripper traced an arc through empty air.

## Where the rock comes from

**One** seed heap of `builder_seed_rock_count = 6`, and that is all the site places. It
covers the collector's first outbound leg, which is minutes of sim long. Everything after
it is delivered by that collector.

`HarvestDropSlot(c) = 6 + 2c` is what makes that work. The harvest lane advances in **wall
slots**, not degrees, so load `c` lands exactly where the wall will have reached — six
seed rocks plus two per earlier load. Each pile is therefore within reach of the station
the builder is already driving to, and the builder clears each pile off the collector
circle before the collector returns to that stretch of it. The step was a flat 30°, which
at 0.03 rad/slot is 17 slots: a two-rock load every 17 slots is a wall that is 88% gaps.

The bridge does not index its feedstock. It takes the **nearest un-consumed rock inside
`[2.0, 5.0]` m of the arm base**, from the seed heap and every delivered load pooled
together. It has to be a search, not a lookup: a load is tipped out of a moving trailer
and lands where it lands. Running both sources through one rule also means the changeover
from heap to delivery needs no code at all — the heap just stops being nearest.

A rock is marked consumed when `StartPickPlace` is issued, not when the arm finishes, so
a failed grab is written off rather than retried. Retrying the rock the arm just failed on
is how a builder spends the rest of a run on one unreachable stone.

`main.cpp` pulls the station **half way** toward the rock on offer, clamped to ±2 slots.
Nominally the two agree, but a tipped load scatters a metre or two, and splitting the
difference keeps the slot (3.1 m inboard) and the rock (~3.9 m outboard) both inside the
5.2 m envelope for an offset either way.

If nothing is in reach the builder holds its slot and logs
`waiting at slot k for N s: M rock(s) known, none within 2.0-5.0 m`. Waiting for the
collector is the normal state between loads; waiting forever is the failure this
arrangement risks, so it is in the log rather than inferred from a wall that stopped
growing.

## Verified reach (scale 2.0, `elbow_up=True`, all `fk_err = 0.00000`)

    pick            reach_xy=3.500  theta=(+1.571,+0.327,-1.209,-1.212)
    place           reach_xy=3.095  theta=(-1.571,+0.366,-1.377,-1.315)
    pick-worst      reach_xy=3.808  theta=(+1.166,+0.287,-1.073,-1.133)
    place-worst     reach_xy=3.377  theta=(-1.159,+0.344,-1.263,-1.250)

All inside the scaled envelope `[2.0, 5.2]`.

## Targets come from the SIM, not from these constants

The table above is how the site is **laid out**. It is *not* how the controller finds its
targets. `pick_target` is read off the rock body and `arm_base_pose` off the arm's base
body, so the IK is solved against where things actually are — the builder is a tracked
vehicle station-keeping on sloped regolith and stops where it stops. That privileged
read is deliberate: the task being demonstrated is construction, not perception.

## Rocks are fixed except the one in the gripper

Seed rocks are created `SetFixed(true)` with collision on; delivered rocks are frozen the
same way once they come to rest, so a couple of dozen extra bodies per rank cost the
solver nothing. `LrvArm::TryLockRock` unfixes exactly one, at the moment the fingers make
contact, and `BuilderArmRosBridge` re-fixes it once laid — it is part of the wall now, and
must not be nudged by the next rock landing beside it. **At most one rock per rank is ever
a dynamic body.**

Two things used to break that for delivered rock and both are gone:

- `RobotRig` *released* a frozen rock when its contact force rose 40 N over its resting
  load, or as a backstop when any builder body came within 3 m. Both fire exactly when the
  arm is trying to take hold, so the rock would be unfixed and rolling downhill at the
  worst possible moment. That rule belonged to the version where a dumped pile was in the
  builder's way; it is now the builder's larder.
- `UpdateRockCollisionActivation` switched collision off on rocks far from the *rover*.
  The rover drives away from its own drop point on the very next cycle, so the pile the
  gripper was about to close on would go non-collidable. It now skips fixed rocks.

## Sector cap

`BuilderWallSlotCount(N)` is now purely the sector cap: 70% of a rank's `2*pi/N`, so one
builder's wall never runs into the next builder's. It is no longer tied to how many heaps
were laid out, because the feedstock is a stream rather than a fixed larder. At N=4 that
is 36 slots (35.6 m of lane); at N=15 it is 9.

## Scale

`RobotArmInverseKinematicsSolver(scale=2.0)` gives `a1..a4 = 0.650, 2.540, 2.286, 0.715`
(5.54 m fully extended). The mesh geometry is scaled 2x but the **finger geometry and the
mass/inertia values stay 1x** (see `BuilderRig`). That split is also why `LrvArm`'s
`min_grab_reach_xy` / `max_grab_reach_xy` / `min_grab_local_z` / `divergence_abort` are
multiplied by `m_geometry_scale` — they are lengths — while every `finger_*` constant and
`lock_finger_dist` are not.
