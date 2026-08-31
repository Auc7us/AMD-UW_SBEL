#!/bin/bash
#SBATCH --job-name=scmfine
#SBATCH --nodes=1
#SBATCH --exclusive
#SBATCH --time=3:55:00
#SBATCH --output=/work1/dannegrut/auc7us/chrono-build-area/batch/%x_%j.out
#SBATCH --error=/work1/dannegrut/auc7us/chrono-build-area/batch/%x_%j.out

# Deformable-SCM demo at a fine grid, unattended. Partition and DELTA come from
# the submitting sbatch line via --export.
#
# -e is set ABOVE what the wall clock can reach on purpose. Both recorders flush
# periodically (poses once per sim second, SCM every frame) and there is no signal
# handler, so letting the wall clock end the run costs at most one sim second and
# means an RTF estimate that is off by 25% does not waste the slot.

set -u
DELTA=${DELTA:-0.03}
RANKS=${RANKS:-5}
ROCKS=${ROCKS:-2}
SIM_END=${SIM_END:-1200}
BASE=/work1/dannegrut/auc7us/chrono-build-area
APP="apptainer exec --rocm -H $BASE /work1/dannegrut/auc7us/chrono.sif"
IN="source ~/mountdir/chrono_env.sh 2>/dev/null; cd ~/mountdir/amd-uw"

echo "=== job $SLURM_JOB_ID  node $SLURMD_NODENAME  $(nproc) cores  $(date) ==="
echo "=== delta=$DELTA ranks=$RANKS (=$((RANKS-1)) robots) rocks/rank=$ROCKS wall=3:55:00 ==="
$APP rocm-smi --showid 2>/dev/null | grep -c "Device ID" | sed 's/^/GPUs visible: /'

# Gate 1: Chrono is built -march=native. A narrower host SIGILLs instantly.
echo "=== gate 1: ISA ==="
if ! $APP bash -c "$IN && ./build/demo_SYN_construction --help >/dev/null 2>&1"; then
    echo "FATAL: binary will not run on $SLURMD_NODENAME (ISA mismatch). Rebuild here. Aborting."; exit 1
fi
echo "gate 1 OK"

# Gate 2: the HIP ray-cast backend falls back to the CPU *silently* when there is no
# code object for this GPU. Run at a coarse delta -- device support does not depend on
# delta, and this way the gate costs ~90 s instead of paging in the fine grid twice.
echo "=== gate 2: does SCM ray-cast reach the GPU on this device? ==="
$APP bash -c "$IN && mpirun --mca plm isolated -np 3 --oversubscribe \
  ./build/demo_SYN_construction --no_sensor -e 2 --no_record --rocks_per_rank 1 \
  --scm_raycast_gpu --scm_delta 0.10 2>&1" > $BASE/batch/gate2_$SLURM_JOB_ID.log 2>&1
grep -E "ray-cast" $BASE/batch/gate2_$SLURM_JOB_ID.log
G=$(grep -oP 'ray-cast: \K[0-9]+' $BASE/batch/gate2_$SLURM_JOB_ID.log | head -1)
if [ "${G:-0}" -lt 100 ]; then
    echo "FATAL: ray-cast fell back to CPU here (${G:-0} GPU steps). At delta=$DELTA the CPU"
    echo "       path is ~10x slower and would waste the slot. Check the HIP target. Aborting."
    exit 1
fi
echo "gate 2 OK ($G GPU steps)"

echo "=== main run ==="
$APP bash $BASE/batch/inner_run.sh "$DELTA" "$SIM_END" "$RANKS" "$ROCKS"
echo "=== main run rc=$? at $(date) ==="
$APP bash -c "$IN && D=\$(ls -dt recordings/run_* | head -1) && echo \"recording: \$D\" && ls -la \$D && cat \$D/rank_1_meta.json"
