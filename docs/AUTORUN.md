# Unattended runs: how to start one, watch it, and pick up after a crash

A long `demo_SYN_construction` run takes 15-40 hours of wall clock and dies partway
through often enough that babysitting it by hand is not viable. This is the procedure
that works, plus the traps that cost time the first few times round.

Read this end to end before starting a run. The stop procedure in particular has two
failure modes that leave orphans behind, and orphans poison the next run.

---

## 1. The shape of the thing

One run is three layers, and all three have to be alive:

| layer | what it is | how many |
|---|---|---|
| supervisor | `supervise_long.sh`, a bash retry loop | 1 |
| sim | `mpirun -np 16 ./build/demo_SYN_construction` | 16 ranks |
| controllers | ROS 2 Humble nodes, 4 per robot | 60 |

Ranks are **not** symmetric. Rank 0 is sensor/orchestrator and owns no robot; ranks
1..15 each own one robot + one builder. So `-np 16` means **15 robots**, and
`robots = np - 1` everywhere in the logs and the layout code.

The four controller types are `pure_pursuit`, `manipulator`, `builder_orbit`,
`builder_arm`. 15 robots x 4 = 60. If the count is not exactly 60, stop and fix it
before launching; a missing controller looks like a frozen robot hours later.

### Ranks cannot see each other

Every run passes `--no_sensor`, and `draws_zombies = is_sensor_rank && !no_sensor`, so
no rank ever builds a body for another rank's machines. Remote agents are registered as
`SynQuietAgent`, whose overrides are all empty. Each rank's `ChSystem` contains only its
own robot, builder, terrain and rocks.

Consequences worth internalising:

- Two ranks' machines **cannot** collide, and cannot interpenetrate either -- in each
  rank's world the other simply is not there.
- A frozen robot on rank N cannot physically affect rank M. Every stall is local.
- Anti-collision between builders therefore lives in the **ROS layer**, not the sim:
  `builder_orbit_controller.py` subscribes to `/builder_{ahead}/vehicle_state` and holds
  at `min_builder_gap_m` (9.0 m). See section 7 for its fail-open behaviour.

---

## 2. Where everything lives

Paths differ between host and container. Getting these wrong wastes a checkpoint.

| | path |
|---|---|
| container name | `chrono-orb` (image `atk/chrono:orb`) |
| container user | `chrono-user`, uid 1000 -- **not root** |
| project, in container | `/home/chrono-user/mountdir` |
| project, on host | `/home/auc7us/sbel/chrono/chrono-docker/mountdir` |
| logs | `mountdir/logs/` |
| **recordings** | `mountdir/amd-uw/recordings/` -- **not** `mountdir/recordings/` |
| supervisor script | `mountdir/supervise_long.sh` (gitignored) |

Two traps here:

- `docker exec chrono-orb ... /root/mountdir` fails with **"Permission denied"**, not
  "no such file". It looks like a mount problem; it is just the wrong user's home.
- The recordings path is the one people get wrong. A `du` against
  `mountdir/recordings/` returns nothing and projects 0 GB, which reads as "recording is
  broken" when it is fine.

**The container clock is UTC. The host is US Central.** Every timestamp in every log is
UTC; subtract 5 h before reporting anything to a human. Log filenames carry the local
launch time, so `sim_20260830_022559_a2.log` began 02:25:59 CDT and its first line says
07:25:59.

---

## 3. Preflight

Do all of these. Each one exists because skipping it cost a run.

```bash
free -g                      # need the machine mostly idle; a run holds ~58 of 62 GB
docker exec chrono-orb bash -lc 'P="amd_uw_r""os2"; pgrep -cf "$P"'   # expect 0
docker exec chrono-orb bash -lc 'pgrep -af supervise_long'            # expect nothing
df -h /home/auc7us                                                    # see section 6
```

- **One simulation at a time.** All controllers share a DDS domain. A second run's
  controllers will talk to the first run's robots. The symptom is duplicate-publisher
  warnings and robots that steer as though possessed.
- **Check for stray supervisors specifically.** A supervisor whose sim you killed by
  hand stays alive and will relaunch on its own schedule. One survived two runs
  undetected in this project's history.
- **Build with `ninja -j4`.** Heavier parallelism has hard-locked this machine. Check
  `free -g` before the build too, not just before the run.

---

## 4. Starting a run

Build first if any source changed, then launch detached inside the container:

```bash
# build (only if sources changed) -- check free -g first
docker exec chrono-orb bash -lc 'cd /home/chrono-user/mountdir/amd-uw/build && ninja -j4'

# launch
TS=$(date +%Y%m%d_%H%M%S)
docker exec -d chrono-orb bash -lc \
  "cd /home/chrono-user/mountdir && ./supervise_long.sh $TS"
```

`supervise_long.sh` then, per attempt: counts controllers and restarts them if under 60,
launches the sim in the foreground (so a crash is noticed the instant it happens),
records `rc` / last sim time / segv count / recording dir, and either stops on `rc=0` or
clears controllers and retries. `MAX=8` attempts. A memory sampler writes
`logs/mem_$TS.log` every 5 minutes throughout.

Sim arguments currently in the script:

```
timeout 400000 mpirun -np 16 -x OMP_NUM_THREADS=1 ./build/demo_SYN_construction \
    --no_sensor --scm_delta 0.035 -e 6000 --perf_log 10 --record_rate 30 --scm_record_rate 10
```

`OMP_NUM_THREADS=1` is deliberate: 16 ranks on 16 cores, one thread each.

### The parameter set

These are the values of the last good run (`run_20260830_073816`, 14 h, 14/15 builders
healthy, 256/435 slots, zero crashes). Start from these; change one thing at a time.

**Sim CLI** -- as in `supervise_long.sh`:

| flag | value | why |
|---|---|---|
| `-np` | `16` | rank 0 + 15 robots |
| `OMP_NUM_THREADS` | `1` | 16 ranks on 16 cores |
| `--no_sensor` | on | headless; also what makes ranks blind to each other |
| `--scm_delta` | `0.035` | terrain grid. The cluster uses `0.02` |
| `-e` | `6000` | max sim seconds |
| `--perf_log` | `10` | perf block every 10 s |
| `--record_rate` | `30` | pose frames/s |
| `--scm_record_rate` | `10` | terrain **delta** frames/s -- see section 6 |
| `timeout` | `400000` | 111 h, so it never binds in practice |

Not passed, so left at defaults: `--rocks_per_rank` (0 = per-rank 2-6, varied),
`--rock_first_distance` (20), `--rock_distance_step` (30), `--rock_mesh_scale` (0.2).

**Controller launch** -- both lines, `robot_ids`/`builder_ids` = `1..15`:

```
robot_controllers.launch.py      robot_ids:=1,..,15  target_speed_mps:=3.0
                                 switch_radius_m:=2.0  rock_side_offset_m:=2.0
builder_orbit_controllers.launch.py  builder_ids:=1,..,15  counter_clockwise:=true
```

**Compiled-in constants that matter** (changing these needs `ninja -j4`):

| constant | file | value |
|---|---|---|
| `scm_keyframe_period` | `main.cpp` | `60.0` s |
| `robot_start_radius` (collector ring) | `RobotLayout.h` | `56.5` m |
| `builder_pile_radius` | `RobotLayout.h` | `56.1` m |
| builder orbit / work circle | `RobotLayout.h` | 53 m / 50 m |
| `wall_slot_pitch_m` | `RobotLayout.h` | `0.5` m |
| `BuilderWallSlotCount` cap | `RobotLayout.h` | `0.7 * sector / pitch` -> 29 slots |
| `feedstock` reach envelope | `BuilderArmRosBridge.cpp` | 2.0 - 5.0 m |
| `station_fetch_max_arc_m` | `BuilderArmRosBridge.cpp` | `3.3` m |
| `station_fetch_min_arc_m` | `BuilderArmRosBridge.cpp` | `0.05` m |
| `starved_slot_timeout` | `BuilderArmRosBridge.cpp` | `240.0` s |
| `collector_keepout_m` / `_max_s` | `BuilderArmRosBridge.cpp` | `4.0` m / `45.0` s |
| `min_builder_gap_m` | `builder_orbit_controllers.launch.py` | `9.0` m |

The collector ring at 56.5 is measured, not guessed: against 57.0 it produced 3x fewer
stuck events per sim-second (0.0121 vs 0.0387) and a worst jackknife of -43 deg against
-102 deg. Keep it unless something better-measured says otherwise.

Rock sizes: one global `--rock_mesh_scale` (0.2) multiplies everything, and
`RockVariantScale` in `RockField.cpp` normalises the three OBJs against each other on
their longest axis (rock2 -> 0.858, rock3 -> 0.917) because the source meshes differ by
up to 1.9x in bulk. Rock size against the 0.41 m wheel clearance is the first thing to
sweep after a jackknife.

### Verify the launch before walking away

```bash
tail -5 logs/supervisor_$TS.log        # want: attempt 1/8, controllers 60/60, launching sim
grep -m2 "keyframe every\|collector ring=" logs/sim_${TS}_a1.log
```

The second line is the point: **confirm the settings you changed are actually in the
running binary.** A fix that never executes is not a fix, and the log echoes the live
values. Verify the controller inventory is exactly 15 of each of the 4 types and that
there are zero duplicate-publisher warnings.

---

## 5. The wake-on-crash setup

Two watches, armed right after launch. They do different jobs.

### Crash monitor -- persistent, fires on every attempt boundary

```
Monitor(
  command: 'S=/home/auc7us/sbel/chrono/chrono-docker/mountdir
            tail -F -n +1 "$S/logs/supervisor_TS.log"',
  description: 'long run -- attempt starts, crashes, rc codes',
  persistent: true)
```

Tailing the **supervisor** log, not the sim log, is what makes this work. The supervisor
writes one line per lifecycle event and nothing else, so every line is worth a
notification and there is no filter to get wrong. Silence means healthy.

### Progress checkpoint -- one-shot, per milestone

Use `Bash` with `run_in_background` and an `until` loop that exits at a sim-time
threshold **or when the sim dies**, so it cannot hang forever after a crash:

```bash
L=logs/sim_${TS}_a2.log
until t=$(grep "^\[0\]:.time=" $L 2>/dev/null | tail -1 | sed 's/.*time=\([0-9.]*\).*/\1/'); \
      [ -n "$t" ] && awk -v x="$t" 'BEGIN{exit !(x>=400)}' 2>/dev/null || \
      ! docker exec chrono-orb pgrep -f demo_SYN_construction >/dev/null 2>&1; do sleep 300; done
# ... then print the checkpoint ...
```

Do **not** use `Monitor` for a one-shot condition -- `tail -f` never exits and the watch
stays armed after the event. Do not use a foreground `sleep`; the harness blocks it.

What a checkpoint should print, and why:

- rank count, `DRIFT` / `DIVERG` / segv / flip counts -- liveness
- recording size + a projection -- disk, see section 6
- **one grep per code path you changed** -- proof the fix executes
- per-builder slot progress -- the real progress metric
- `free -g` and `df -h`

Pick thresholds where something is expected to happen. Early builder slots come off a
6-rock seed heap and go fast; everything after waits on collector round trips, so
fetch/starvation behaviour does not appear until t~300+.

### When a crash notification arrives

1. `tail -40` the sim log. Classify: `SIGSEGV`, `DIVERGED`, or clean `rc=0` completion.
2. For a `DIVERGED`, read **upward** from the abort. The report names the first
   non-finite body and its link reactions, and the lines above it are the real story
   (`kingpin`, `DRIFT`, `STUCK`). A divergence with **no** preceding `DIVERGING` warning
   blew up in a single step -- that is a solver failure, not a scene event, and there is
   no scene fix for it.
3. Check whether it reproduces: compare against the previous attempt's log at the same
   sim time. This project has seen the same binary diverge at t=16.8 in one attempt and
   sail past in the next. Do not attribute a nondeterministic divergence to your last
   source change without checking that the change can even affect physics.
4. The supervisor has already retried. Only intervene if the fix is certain.

---

## 6. Recording size -- read this before changing record rates

There are two independent knobs and they are wildly asymmetric:

| knob | where | what it controls |
|---|---|---|
| `--scm_record_rate` | CLI, `main.cpp:98` | how often **delta** frames are written |
| `scm_keyframe_period` | `main.cpp:115`, compiled in | seconds between **keyframes** |

Measured on a completed 1500 s run at 1 Hz / 5 s keyframes, rank 12:

```
file        8.05 GB, 1500 frames, 670,468,513 node rows
keyframes    300 frames, 666,437,040 rows  = 99.4 %
deltas      1200 frames,   4,031,473 rows  =  0.6 %
```

**Keyframes are effectively the entire file.** Each one re-states every deformed node,
and the deformed set only grows, so total size is quadratic in run length. Lowering the
sample rate to save disk is close to useless -- it shrinks the 0.6 %.

`scm_keyframe_period` is a **period in seconds**: smaller means *more* keyframes. Going
5 -> 60 cut a 520 GiB projection to ~46 GB; going 5 -> 0.5 would have made it ~5 TiB.

Current settings are 10 Hz deltas with 60 s keyframes: ten times finer time resolution
than the old default for about a tenth of the bytes.

Sanity-check disk at the first checkpoint rather than trusting the projection. Fit two
size samples to `a*t + b*t^2`; a purely linear extrapolation understates the tail.

---

## 7. Known failure modes

**Collector jackknife (terminal).** The collector wedges its trailer against its own
dumped rocks and pins the throttle. The `STUCK` detector reports it in full -- hitch
articulation, per-wheel `Fz`, rim speed. Signature is `throttle=1.00, speed=0.00,
guard_limited=0.00%` with steering winding up as the controller saws for an escape.
**Nothing acts on the detector**; the rank is lost for the rest of the run, and its
builder starves forever. `guard_limited` is the *traction* guard (throttle vs slip) --
there is no anti-collision guard in the C++ at all.

**Builder starved at a slot.** Look at the `status: slot` line: `nearest` outside the
2.0-5.0 m envelope means the rock is unreachable. Too far is handled by a station slide
(arc only -- a purely radial miss cannot be closed by sliding along an orbit). Too close
has no handler. Both are supposed to fall through to `starved_slot_timeout`.

**Divergence with no precursor.** Nominal to 1e97 in ~9 timesteps, no `DIVERGING`
warning first. Unexplained. Retry; it usually does not recur at the same time.

**Log spam.** Any per-step diagnostic without a rate limit will emit tens of lines per
sim-second. One unthrottled branch produced 955 identical lines in 62 s of sim. Gate
every new diagnostic on a report period.

**Fail-open neighbour fence.** `gap_to_ahead()` returns `None` when the neighbour's
topic is silent, and `None` means *no constraint* -- deliberately, so a crashed rank does
not halt the site. Safe while `BuilderWallSlotCount`'s 0.7 sector cap keeps builders
apart by construction. If that cap is ever raised, the fence becomes the only protection
and must be made fail-closed first.

---

## 8. Stopping a run

**Order matters: supervisor, then sim, then controllers.** Kill the sim first and the
supervisor cheerfully launches attempt N+1.

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

- **`pkill -f` matches its own shell.** The pattern appears in the `bash -lc` command
  line, so `pkill -9 -f amd_uw_ros2` kills the shell running it -- exit 137, controllers
  untouched, and it *looks* like the kill worked. Split the literal
  (`P="amd_uw_r""os2"`) so the assembled string never appears in the command line.
- **Host `kill -9` returns "Permission denied"** for container processes under the tool
  sandbox. Kill from inside via `docker exec`, using container PIDs.

Verify with `free -g` -- a stopped run drops usage from ~58 GB to ~5 GB. Then stop the
Monitor task (`TaskStop`) so it does not linger for the session.

---

## 9. Reading progress

Per-builder wall progress, the metric that actually matters:

```bash
grep "status: slot" $L | tail -15 | \
  sed 's/.*chrono_\(builder_[0-9]*\)_arm.: t=[0-9.]* status: slot \([0-9]*\)\/.*/\1 \2/' | sort
```

`BuilderWallSlotCount` = `0.7 * sector_rad / wall_slot_pitch_rad`, so with 15 builders
each owns **29 slots** (14.5 m of a 20.9 m sector) and 435 site-wide. The 0.7 is why a
builder stops ~6.4 m short of its neighbour, and why **no run length produces a closed
loop** -- the walls are built not to touch, by construction.

Real-time factor is ~42x (wall/sim), degrading ~11 % per 600 s of sim as placed rocks
accumulate in the contact set. Budget from the *recent* slot rate, not the run average:
early slots come off the seed heap and flatter the projection badly.
