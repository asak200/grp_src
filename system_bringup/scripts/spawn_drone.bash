#!/usr/bin/env bash
set -e

############ required paths
# px4
PX4_DIR=~/PX4-Autopilot

# px4 build
BUILD_DIR=$PX4_DIR/build/px4_sitl_default

# gazebo world
WORLD=empty

# model name (must match airframe config)
MODEL=${1:-iris_camera} # use iris as default

# spawn coordinate
SPAWN_X=${2:-2}
SPAWN_Y=${3:-0}
SPAWN_Z=${4:-2}


# ENVIRONMENT SETUP

echo "[INFO] Sourcing Gazebo + PX4 environment"

source $PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash \
       $PX4_DIR \
       $BUILD_DIR


############ REQUIRED PX4 VARIABLES
export PX4_SIM_MODEL=gazebo-classic_iris
export PX4_SIM_WORLD=$WORLD

# Optional but recommended
export ROS_VERSION=2
echo
echo
# ############ START GAZEBO SERVER
# echo "[INFO] Starting gzserver"
# gzserver --verbose $WORLD.world &
# SIM_PID=$!

# # Give Gazebo time to initialize
# sleep 3

# ############ START GAZEBO GUI
# echo "[INFO] Starting gzclient"
# gzclient --verbose &
# GUI_PID=$!

############ SPAWN VEHICLE
# note: 
# if you're using a custom model,
# you should make a file for the custome model in
# ~/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes
# I did the iris_camera model just by copying the iris model and rename the file

MODEL_PATH=$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models

echo "[INFO] Spawning model: $MODEL at x=$SPAWN_X y=$SPAWN_Y z=$SPAWN_Z"
gz model --verbose \
  --spawn-file $MODEL_PATH/$MODEL/$MODEL.sdf \
  --model-name $MODEL \
  -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z
echo
echo

############ START PX4 SITL
echo "[INFO] Starting PX4 SITL"

cd $BUILD_DIR/rootfs

$BUILD_DIR/bin/px4 $BUILD_DIR/etc

# CLEANUP ON EXIT
echo
echo "[INFO] Cleaning up"
if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
    kill -9 "$SIM_PID"
fi

if [[ -n "$GUI_PID" ]] && kill -0 "$GUI_PID" 2>/dev/null; then
    kill -9 "$GUI_PID"
fi
