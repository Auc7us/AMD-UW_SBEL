# Builder build cycle

Working note for `builder_arm_controller.py`, `BuilderArmRosBridge` and the build-plan
block in `RobotLayout.h`. Kept because the geometry is not obvious from the code and
getting it wrong has twice produced an arm that waved at nothing.

## What the builder does

It builds. On its own. The loop is one wall slot long:

1. The sim publishes `pick_target` with `ready=1` — meaning the hull is **parked and
   pinned**, slot `k` is unlaid, and its feedstock rock is still in the heap.
2. This node solves grab and place IK **in the arm base frame** and sends one
   12-element `arm_cmd`.
3. `LrvArm`'s pick/place state machine runs it: approach, close on contact, lock, lift,
   swing, settle, release, stow.
4. The bridge re-fixes the laid rock and increments the placed count.
5. `main.cpp` turns that count into the next station angle. The orbit controller creeps
   ~1 m. Then `ready` goes high again for slot `k+1`.

Nothing in that loop reads the collector's harvest cycle, its dump, or its mission
state. Previously `SetStationAngle` was `RankRayAngleRad(..., robot->GetHarvestCycle())`,
so the builder sat still for a whole harvest and then jumped 30° in one step.

## The geometry, all from one integer

`k` is the wall slot index. `ray` is the rank's site ray. `arm_lead = atan(2.5/33) =
0.0757 rad`.

| | radius | angle | distance from arm base |
|---|---|---|---|
| wall slot `k` | 30.0 | `ray + k*pitch` | 3.095 m |
| **station** for `k` | 33.0 | `ray + k*pitch + arm_lead` | — |
| arm base when on station | 33.095 | `ray + k*pitch` | — |
| heap for `k` | 36.6 | station angle at the heap's centre | ~3.5 m |

`pitch = 0.9 m / 30 m = 0.03 rad`, so one slot is **0.9 m of course and 0.99 m of lane**.

`arm_lead` is why the table works: the arm mounts 2.5 m **back** along a hull parked
tangentially, so adding it to the station puts the *arm base*, not the hull origin,
radially opposite the slot it is serving. Both reaches then land mid-band automatically.

Arm `+Y` is radially **outward** (`arm_rot = chassis_rot * QuatFromAngleZ(CH_PI)` in
`BuilderRig`), so pick and place are 180° apart in `theta1` — measured `+1.571` /
`-1.571`. An early version swung 0.55 rad between hand-picked angles, which is why the
gripper traced an arc through empty air.

One heap serves `wall_slots_per_pile = 4` consecutive slots and sits at the middle of
that run, so it is never more than ~1.5 m of lane from the arm base working it.

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

Every feedstock rock is created `SetFixed(true)` with collision on, so a couple of dozen
extra bodies per rank cost the solver nothing. `LrvArm::TryLockRock` unfixes exactly one,
at the moment the fingers make contact, and `BuilderArmRosBridge` re-fixes it once laid —
it is part of the wall now, and must not be nudged by the next rock landing beside it.
**At most one rock per rank is ever a dynamic body.**

## Sector cap

`BuilderWallSlotCount(N)` caps the course at 70% of a rank's `2*pi/N` sector so one
builder's wall never runs into the next builder's. At N=2 that leaves the full 24 slots;
at N=15 it is 9.

## Scale

`RobotArmInverseKinematicsSolver(scale=2.0)` gives `a1..a4 = 0.650, 2.540, 2.286, 0.715`
(5.54 m fully extended). The mesh geometry is scaled 2x but the **finger geometry and the
mass/inertia values stay 1x** (see `BuilderRig`). That split is also why `LrvArm`'s
`min_grab_reach_xy` / `max_grab_reach_xy` / `min_grab_local_z` / `divergence_abort` are
multiplied by `m_geometry_scale` — they are lengths — while every `finger_*` constant and
`lock_finger_dist` are not.
