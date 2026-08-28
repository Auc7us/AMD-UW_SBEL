#!/bin/bash
DELTA=$1; SIM_END=$2; RANKS=$3; ROCKS=$4
source /opt/ros/humble/setup.bash || exit 1
source ~/mountdir/chrono_env.sh 2>/dev/null
export PYTHONUSERBASE=$HOME/.local
export PYTHONPATH=$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$((70 + RANDOM % 20))
cd ~/mountdir/amd-uw/ros2_ws && source install/setup.bash || exit 1
cd ~/mountdir/amd-uw || exit 1
L=~/mountdir/amd-uw/batch_logs; mkdir -p $L
IDS=$(seq -s, 1 $((RANKS-1)))
echo "controllers for ids: $IDS"

ros2 launch amd_uw_ros2 robot_controllers.launch.py robot_ids:=$IDS \
  target_speed_mps:=3.0 switch_radius_m:=2.0 rock_side_offset_m:=2.0 > $L/collectors_$SLURM_JOB_ID.log 2>&1 &
C1=$!
ros2 launch amd_uw_ros2 builder_orbit_controllers.launch.py builder_ids:=$IDS \
  work_circle_radius_m:=50.0 path_radius_m:=53.0 counter_clockwise:=true > $L/builders_$SLURM_JOB_ID.log 2>&1 &
C2=$!
sleep 25
echo "controller nodes up: $(ros2 node list 2>/dev/null | wc -l)"

# 3h40m of the 3h55m wall, leaving time for the recording listing afterwards.
timeout -s INT 13200 stdbuf -oL -eL \
  mpirun --mca plm isolated -np "$RANKS" --oversubscribe \
  ./build/demo_SYN_construction --no_sensor -e "$SIM_END" --perf_log 60 \
  --rocks_per_rank "$ROCKS" --scm_raycast_gpu --scm_delta "$DELTA"
RC=$?
echo "DEMO_EXIT=$RC at $(date)"
kill $C1 $C2 2>/dev/null; sleep 3
pkill -f amd_uw_ros2; pkill -f pure_pursuit; pkill -f builder_orbit
pkill -f builder_arm; pkill -f manipulator
exit $RC
