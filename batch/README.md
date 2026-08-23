# Unattended fine-grid SCM runs

`scm_run.sbatch` is the whole job; `inner_run.sh` is the part that runs inside the
container (ROS 2 controllers up, sim, teardown). Paths assume the HPC Fund layout
`/work1/dannegrut/auc7us/chrono-build-area` with `chrono.sif` beside it.

    sbatch -p mi3001x --job-name=scm3cm \
      --export=ALL,DELTA=0.03,RANKS=5,ROCKS=2,SIM_END=1200 scm_run.sbatch

`RANKS` is total MPI ranks: rank 0 orchestrates and owns no robot, so `RANKS=5`
is four robots. Layout stays collision-free at any count because
`BuilderWallSlotCount` caps each builder at 0.7 of its own `2*pi/N` sector.

`SIM_END` is deliberately set beyond what the wall clock can reach. Both recorders
flush periodically -- poses once per sim second, SCM every frame -- and there is no
signal handler, so letting the wall clock end the run costs at most one sim second.
That means an RTF estimate that is off by 25% does not waste the slot.

Two gates run before the long run, so a bad node fails in two minutes rather than
four hours:

  1. ISA. Chrono is built `-march=native` on a compute node; a narrower host
     SIGILLs immediately (this is what the login node does).
  2. GPU ray-cast. The HIP backend falls back to the CPU *silently* when there is
     no code object for the device, and at these deltas the CPU path is ~10x
     slower. The gate runs at delta=0.10 because device support does not depend on
     delta, and aborts unless the ray-cast counter clears 100 steps.

Measured scaling, one node, GPU ray-cast, 3 ranks, controllers driving:

    wall/sim ~= 14.7 + 0.96 * (0.10/delta)^2

Core count does not enter it. At delta=0.10, 16 cores gives 15.68 wall/sim against
16.38 on 192 -- the demo is three largely serial MPI ranks, and with ray casting on
the GPU what is left on the CPU is the per-rank multibody solve. A single GPU is
likewise enough; every rank lands on device 0 at 9% utilisation.
