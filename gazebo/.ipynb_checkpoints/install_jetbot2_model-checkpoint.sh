#!/bin/bash

#
# make sure gazebo has been installed first
#
GAZEBO_DIR=~/.gazebo
GAZEBO_MODEL_DIR=$GAZEBO_DIR/models
GAZEBO_JETBOT2_DIR=$GAZEBO_MODEL_DIR/jetbot2

echo "checking for:  " $GAZEBO_DIR

if [ ! -d "$GAZEBO_DIR" ]; then
	echo "error:  please install and run Gazebo once before running this script"
	exit
fi

echo "checking for:  "  $GAZEBO_MODEL_DIR

if [ ! -d "$GAZEBO_MODEL_DIR" ]; then
	echo "error:  please install and run Gazebo once before running this script"
	exit
fi

echo "checking for:  " $GAZEBO_JETBOT2_DIR

if [ -d "$GAZEBO_JETBOT2_DIR" ]; then
	echo "error:  $GAZEBO_JETBOT2_DIR already exists"
	exit
fi

#
# get the directory of the jetbot2 model
#
SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
JETBOT2_DIR=$SOURCE_DIR/jetbot2

echo "source from:   " $SOURCE_DIR
echo "jetbot2 from:   " $JETBOT2_DIR
echo " "
echo "linking $GAZEBO_JETBOT2_DIR -> $JETBOT2_DIR" 

ln -s $JETBOT2_DIR $GAZEBO_JETBOT2_DIR
