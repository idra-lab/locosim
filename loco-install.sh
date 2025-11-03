#!/bin/bash

ubuntu_version=0
install_realsense_camera=false
install_graspin=false

# Parse command-line options
while getopts ":u:hrg" option; do
  case $option in
	u) ubuntu_version=$OPTARG;;
    r) install_realsense_camera=true;;
    g) install_graspin=true;;
    h)
      echo "Usage: loco-install -u <ubuntu_version> [-r] [-g] [-h]"
      echo "  -u <ubuntu_version>: Supported versions are 18 or 20."
      echo "  -r: Install RealSense camera packages."
      echo "  -g: Install GraspIn packages."
      echo "  -h: Display this help message."
      echo "Note: Options can be placed before or after the version number."
      exit 0
      ;;
    \?) # Handle invalid options
      echo "Usage: loco-install -u <ubuntu_version> [-r] [-g] [-h]"
      exit 1
      ;;
  esac
done

# Check Ubuntu supported version
if [ "$ubuntu_version" != "18" ] && [ "$ubuntu_version" != "20" ]; then
	echo "Ubuntu versions supported are 18 or 20"
	exit 1
fi

# Set up prefix to use for installation
if [ "$ubuntu_version" == "18" ]; then
	PYTHON_PREFIX="python3"
	PYTHON_VERSION="3.5"
	ROBOTPKG_PYTHON_VERSION="py35"
	PIP_PREFIX="pip3"
	ROS_VERSION="bionic"
elif [ "$ubuntu_version" == "20" ]; then
	PYTHON_PREFIX="python3"
    PYTHON_VERSION="3.8"
    ROBOTPKG_PYTHON_VERSION="py38"
    PIP_PREFIX="pip3"
    ROS_VERSION="noetic"
fi

# Install base libraries
sudo apt install -qqy lsb-release gnupg2 curl

# Adds repositories
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ] || \
! grep -q "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" /etc/apt/sources.list.d/ros-latest.list; \
then
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi

if [ ! -e /etc/apt/sources.list.d/robotpkg.list ] || \
! grep -q "deb \[arch=amd64\] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg" "/etc/apt/sources.list.d/robotpkg.list"; \
then
	sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
fi

if [ ! -e /etc/apt/sources.list.d/robotpkg.list ] || \
! grep -q "deb \[arch=amd64\] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg" /etc/apt/sources.list.d/robotpkg.list ; \
then
	sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
fi

# Adds apt keys
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -

# Download packages
sudo apt install -y "ros-$ROS_VERSION-desktop-full"

sudo apt install -y "ros-$ROS_VERSION-urdfdom-py" \
"ros-$ROS_VERSION-srdfdom" \
"ros-$ROS_VERSION-joint-state-publisher" \
"ros-$ROS_VERSION-joint-state-publisher-gui" \
"ros-$ROS_VERSION-joint-state-controller" \
"ros-$ROS_VERSION-gazebo-msgs" \
"ros-$ROS_VERSION-control-toolbox" \
"ros-$ROS_VERSION-gazebo-ros" \
"ros-$ROS_VERSION-controller-manager" \
"ros-$ROS_VERSION-joint-trajectory-controller" \ 
"ros-$ROS_VERSION-catkin"

if [ $install_realsense_camera = true ]; then
	sudo apt install -y "ros-$ROS_VERSION-openni2-launch" \
	"ros-$ROS_VERSION-openni2-camera" \
	"ros-$ROS_VERSION-realsense2-description"
fi

if [ $install_graspin = true ]; then
	sudo apt install -y "ros-$ROS_VERSION-eigen-conversions" \
	"ros-$ROS_VERSION-object-recognition-msgs" \
	"ros-$ROS_VERSION-roslint"
fi

sudo apt update -y

sudo apt install -y "robotpkg-$ROBOTPKG_PYTHON_VERSION-eigenpy" \
"robotpkg-$ROBOTPKG_PYTHON_VERSION-pinocchio" \
"robotpkg-$ROBOTPKG_PYTHON_VERSION-quadprog"

sudo apt install -y "$PYTHON_PREFIX-scipy" \
"$PYTHON_PREFIX-matplotlib" \
"$PYTHON_PREFIX-termcolor" \
"$PYTHON_PREFIX-pip" \
"$PYTHON_PREFIX-catkin-tools"

$PIP_PREFIX install cvxpy==1.2.0

if ! grep -q "source /opt/ros/$ROS_VERSION/setup.bash" ~/.bashrc; then
	echo "source /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
fi
