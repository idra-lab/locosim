#!/bin/bash

i_option=false
while getopts "p:c:ih" option; do
  case $option in
	p) package_name=$OPTARG;;
	c) controller_name=$OPTARG;;
	i) i_option=true;;
    *)
      echo "Usage: loco-run -p <package_name> -c <controller_name> [-i]"
      exit 1
      ;;
  esac
done

if [ -z "$package_name" ]; then 
	echo "Package name not found"
	exit 1
fi

if [ -z "$controller_name" ]; then 
	echo "Controller name not found"
	exit 1
fi

if [ -d "$LOCOSIM_DIR/robot_descriptions/$package_name/controller" ]; then
	cp $LOCOSIM_DIR/robot_descriptions/$package_name/controller/* $LOCOSIM_DIR/robot_control/base_controllers || exit 1
	catkin build $package_name --workspace $HOME/ros_ws
else
	echo "WARNING: Package controller directory is not found"
fi

if [ "$i_option" = true ]; then
    python3 -i $LOCOSIM_DIR/robot_control/base_controllers/$controller_name
else 
    python3 $LOCOSIM_DIR/robot_control/base_controllers/$controller_name
fi
