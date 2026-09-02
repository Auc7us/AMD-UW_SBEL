# Unattended runs

A long `demo_SYN_construction` run takes 15–40 h of wall clock and dies partway through often
enough that babysitting it by hand is not viable. Read this end to end before starting one — the
stop procedure has two failure modes that leave orphans behind, and orphans poison the next run.

## 1. The shape of the thing

Three layers, all of which have to be alive:

| layer | what it is | how many |
|---|---|---|
| supervisor | `supervise_long.sh`, a bash retry loop | 1 |
| sim | `mpirun -np 16 ./build/demo_SYN_construction` | 16 ranks |
| controllers | ROS 2 Humble nodes, 4 per robot | 60 |

Ranks are **not** symmetric: rank 0 is sensor/orchestrator and owns no robot, ranks 1..15 each
own one robot + one builder. The four controller types are `pure_pursuit`, `manipulator`,
`builder_orbit`, `builder_arm`. 15 × 4 = 60. If the count is not exactly 60, fix it before
launching; a missing controller looks like a frozen robot hours later.

**Ranks cannot see each other.** Every run passes `--no_sensor` and
`draws_zombies = is_sensor_rank && !no_sensor`, so remote agents are `SynQuietAgent` and each
rank's `ChSystem` holds only its own machines. Two ranks' machines cannot collide or
interpenetrate, and every stall is local. Anti-collision between builders therefore lives in the
ROS layer — see section 6.

## 2. Where everything lives

| | path |
|---|---|
| container name | `chrono-orb` (image `atk/chrono:orb`) |
| container user | `chrono-user`, uid 1000 — **not root** |
| project, in container | `/home/chrono-user/mountdir` |
| project, on host | `/home/auc7us/sbel/chrono/chrono-docker/mountdir` |
| logs | `mountdir/logs/` |
| **recordings** | `mountdir/amd-uw/recordings/` — **not** `mountdir/recordings/` |
| supervisor script | `mountdir/supervise_long.sh` (gitignored) |

Two traps: `docker exec chrono-orb ... /root/mountdir` fails with **"Permission denied"**, not
"no such file" — it is the wrong user's home, not a mount problem. And a `du` against
`mountdir/recordings/` returns nothing and projects 0 GB, which reads as "recording is broken"
when it is fine.

**The container clock is UTC, the host is US Central.** Subtract 5 h before reporting anything to
a human. Log filenames carry the local launch time, so `sim_20260830_022559_a2.log` began
02:25:59 CDT and its first line says 07:25:59.

## 3. Preflight

Each of these exists because skipping it cost a run.

```bash
free -g                      # need the machine mostly idle; a run holds ~58 of 62 GB
docker exec chrono-orb bash -lc 'P="amd_uw_r""os2"; pgrep -cf "$P"'   # expect 0
docker exec chrono-orb bash -lc 'pgrep -af supervise_long'            # expect nothing
df -h /home/auc7us                                                    # see section 5
```

- **One simulation at a time.** All controllers share a DDS domain, so a second run's controllers
  will talk to the first run's robots. The symptom is duplicate-publisher warnings and robots
  that steer as though possessed.
- **Check for stray supervisors specifically.** A supervisor whose sim you killed by hand stays
  alive and relaunches on its own schedule. One survived two runs undetected.
- **Build with `ninja -j4`.** Heavier parallelism has hard-locked this machine.

## 4. Starting a run

```bash
# build (only if sources changed) -- check free -g first
docker exec chrono-orb bash -lc 'cd /home/chrono-user/mountdir/amd-uw/build && ninja -j4'

TS=$(date +%Y%m%d_%H%M%S)
docker exec -d chrono-orb bash -lc \
  "cd /home/chrono-user/mountdir && ./supervise_long.sh $TS"
```

Per attempt, `supervise_long.sh` counts controllers and restarts them if under 60, launches the
sim in the foreground (so a crash is noticed instantly), records `rc` / last sim time / segv count
/ recording dir, and either stops on `rc=0` or clears controllers and retries. `MAX=8` attempts.
A memory sampler writes `logs/mem_$TS.log` every 5 minutes.

```
timeout 400000 mpirun -np 16 -x OMP_NUM_THREADS=1 ./build/demo_SYN_construction \
    --no_sensor --scm_delta 0.035 -e 6000 --perf_log 10 --record_rate 30 --scm_record_rate 10
```

### The parameter set

Values of the last good run (`run_20260830_073816`, 14 h, 14/15 builders healthy, 256/435 slots,
zero crashes). Start from these; change one thing at a time.

| flag | value | why |
|---|---|---|
| `-np` | `16` | rank 0 + 15 robots |
| `OMP_NUM_THREADS` | `1` | 16 ranks on 16 cores |
| `--no_sensor` | on | headless; also what makes ranks blind to each other |
| `--scm_delta` | `0.035` | terrain grid. The cluster uses `0.02` |
| `-e` | `6000` | max sim seconds |
| `--perf_log` | `10` | perf block every 10 s |
| `--record_rate` | `30` | pose frames/s |
| `--scm_record_rate` | `10` | terrain **delta** frames/s — see section 5 |
| `timeout` | `400000` | 111 h, so it never binds in practice |

Left at defaults: `--rocks_per_rank` (0 = per-rank 2–6), `--rock_first_distance` (20),
`--rock_distance_step` (30), `--rock_mesh_scale` (0.2). Controllers launch with
`robot_ids`/`builder_ids` = `1..15`, `target_speed_mps:=3.0`, `switch_radius_m:=2.0`,
`rock_side_offset_m:=2.0`, `counter_clockwise:=true`.

Compiled-in constants that matter (changing these needs `ninja -j4`):

| constant | file | value |
|---|---|---|
| `scm_keyframe_period` | `main.cpp` | `60.0` s |
| `robot_start_radius` (collector ring) | `RobotLayout.h` | `56.5` m |
| `builder_pile_radius` | `RobotLayout.h` | `56.1` m |
| builder orbit / work circle | `RobotLayout.h` | 53 m / 50 m |
| `wall_slot_pitch_m` | `RobotLayout.h` | `0.5` m |
| `BuilderWallSlotCount` | `RobotLayout.h` | flat `200` slots, **no sector cap** |
| `feedstock` reach envelope | `BuilderArmRosBridge.cpp` | 2.0 – 5.0 m |
| `station_fetch_max_arc_m` / `_min_` | `BuilderArmRosBridge.cpp` | `3.3` m / `0.05` m |
| `starved_slot_timeout` | `BuilderArmRosBridge.cpp` | `240.0` s |
| `collector_keepout_m` / `_max_s` | `BuilderArmRosBridge.cpp` | `4.0` m / `45.0` s |
| `min_builder_gap_m` | `builder_orbit_controllers.launch.py` | `9.0` m |

The collector ring at 56.5 is measured: against 57.0 it produced 3x fewer stuck events per
sim-second (0.0121 vs 0.0387) and a worst jackknife of -43° against -102°.

Rock sizes: one global `--rock_mesh_scale` (0.2) multiplies everything, and `RockVariantScale` in
`RockField.cpp` normalises the three OBJs against each other on their longest axis (rock2 →
0.858, rock3 → 0.917) because the source meshes differ by up to 1.9x in bulk. Rock size against
the 0.41 m wheel clearance is the first thing to sweep after a jackknife.

### Verify the launch before walking away

```bash
tail -5 logs/supervisor_$TS.log        # want: attempt 1/8, controllers 60/60, launching sim
grep -m2 "keyframe every\|collector ring=" logs/sim_${TS}_a1.log
```

The second line is the point: **confirm the settings you changed are in the running binary.** A
fix that never executes is not a fix. Verify exactly 15 of each of the 4 controller types and
zero duplicate-publisher warnings.

## 5. Watching a run

**Crash monitor** — tail the *supervisor* log, not the sim log. The supervisor writes one line per
lifecycle event and nothing else, so every line is worth a notification and there is no filter to
get wrong. Silence means healthy.

**Progress checkpoints** — a background `until` loop that exits at a sim-time threshold **or when
the sim dies**, so it cannot hang forever after a crash:

```bash
L=logs/sim_${TS}_a2.log
until t=$(grep "^\[0\]:.time=" $L 2>/dev/null | tail -1 | sed 's/.*time=\([0-9.]*\).*/\1/'); \
      [ -n "$t" ] && awk -v x="$t" 'BEGIN{exit !(x>=400)}' 2>/dev/null || \
      ! docker exec chrono-orb pgrep -f demo_SYN_construction >/dev/null 2>&1; do sleep 300; done
```

A checkpoint should print rank count, `DRIFT`/`DIVERG`/segv/flip counts, recording size plus a
projection, **one grep per code path you changed**, per-builder slot progress, `free -g` and
`df -h`. Pick thresholds where something is expected to happen: early builder slots come off the
6-rock seed heap and go fast, so fetch/starvation behaviour does not appear until t~300+.

When a crash notification arrives:

1. `tail -40` the sim log. Classify: `SIGSEGV`, `DIVERGED`, or clean `rc=0`.
2. For a `DIVERGED`, read **upward** from the abort. The report names the first non-finite body;
   the lines above it are the real story (`kingpin`, `DRIFT`, `STUCK`). A divergence with **no**
   preceding `DIVERGING` warning blew up in one step — a solver failure, not a scene event, and
   there is no scene fix for it.
3. Check whether it reproduces. The same binary has diverged at t=16.8 in one attempt and sailed
   past in the next; do not attribute a nondeterministic divergence to your last source change
   without checking the change can even affect physics.
4. The supervisor has already retried. Only intervene if the fix is certain.

### Recording size

Two knobs, wildly asymmetric: `--scm_record_rate` (CLI) sets how often **delta** frames are
written; `scm_keyframe_period` (compiled in) sets seconds between **keyframes**. Measured on a
1500 s run at 1 Hz / 5 s keyframes, rank 12:

```
file        8.05 GB, 1500 frames, 670,468,513 node rows
keyframes    300 frames, 666,437,040 rows  = 99.4 %
deltas      1200 frames,   4,031,473 rows  =  0.6 %
```

**Keyframes are effectively the entire file.** Each re-states every deformed node and the deformed
set only grows, so total size is quadratic in run length. Lowering the sample rate shrinks the
0.6%. `scm_keyframe_period` is a period in seconds, so smaller means *more* keyframes: 5 → 60 cut
a 520 GiB projection to ~46 GB; 5 → 0.5 would have made it ~5 TiB. Sanity-check disk at the first
checkpoint rather than trusting the projection — fit two size samples to `a*t + b*t^2`, since a
linear extrapolation understates the tail.

## 6. Known failure modes

**Collector jackknife (terminal).** The collector wedges its trailer against its own dumped rocks
and pins the throttle. Signature is `throttle=1.00, speed=0.00, guard_limited=0.00%` with steering
sawing for an escape. The `STUCK` detector reports it in full, but **nothing acts on it**: the
rank is lost for the rest of the run and its builder starves forever. `guard_limited` is the
*traction* guard (throttle vs slip); there is no anti-collision guard in the C++ at all.

**Builder starved at a slot.** On the `status: slot` line, `nearest` outside the 2.0–5.0 m
envelope means the rock is unreachable. Too far is handled by a station slide (arc only — a purely
radial miss cannot be closed by sliding along an orbit); too close has no handler. Both should
fall through to `starved_slot_timeout`.

**Divergence with no precursor.** Nominal to 1e97 in ~9 timesteps, no `DIVERGING` warning first.
Unexplained. Retry; it usually does not recur at the same time.

**Log spam.** Any per-step diagnostic without a rate limit emits tens of lines per sim-second. One
unthrottled branch produced 955 identical lines in 62 s of sim. Gate every new diagnostic on a
report period.

**Builder-to-builder collision.** `BuilderWallSlotCount` no longer caps a builder to its own
sector, so `min_builder_gap_m` (9.0 m) is the *only* thing keeping two builders apart — and
`gap_to_ahead()` is fail-open, returning `None` (no constraint) when the neighbour's topic goes
quiet. That is deliberate, so a crashed rank does not halt the site, but it means a silent
neighbour is an unfenced one. See DESIGN.md.

## 7. Stopping a run

**Order matters: supervisor, then sim, then controllers.** Kill the sim first and the supervisor
cheerfully launches attempt N+1.

```bash
# 1. supervisors -- ALL of them, including strays from earlier runs
docker exec chrono-orb bash -lc 'pgrep -af supervise_long'
docker exec chrono-orb bash -lc 'kill -9 <pids>'

# 2. sim
docker exec chrono-orb bash -lc 'R="demo_SYN_cons""truction"; pkill -9 -f "$R"'

# 3. controllers
docker exec chrono-orb bash -lc 'P="amd_uw_r""os2"; pkill -9 -f "$P"
                                 Q="ros2 lau""nch";  pkill -9 -f "$Q"'
```

Two traps, both of which have bitten:

- **`pkill -f` matches its own shell.** The pattern appears in the `bash -lc` command line, so
  `pkill -9 -f amd_uw_ros2` kills the shell running it — exit 137, controllers untouched, and it
  *looks* like the kill worked. Split the literal (`P="amd_uw_r""os2"`) so the assembled string
  never appears in the command line.
- **Host `kill -9` returns "Permission denied"** for container processes under the tool sandbox.
  Kill from inside via `docker exec`, using container PIDs.

Verify with `free -g` — a stopped run drops usage from ~58 GB to ~5 GB.

## 8. Reading progress

```bash
grep "status: slot" $L | tail -15 | \
  sed 's/.*chrono_\(builder_[0-9]*\)_arm.: t=[0-9.]* status: slot \([0-9]*\)\/.*/\1 \2/' | sort
```

Each builder may lay up to 200 slots, so the wall is not sector-bounded and a long run has
builders laying over their neighbours' finished work by design.

Real-time factor is ~42x (wall/sim), degrading ~11% per 600 s of sim as placed rocks accumulate in
the contact set. Budget from the *recent* slot rate, not the run average: early slots come off the
seed heap and flatter the projection badly.
