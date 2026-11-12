# Install dependencies

### UBUNTU VERSIONS:

Locosim is compatible with Ubuntu 18/20. The installation instructions have been generalized accordingly. You need replace four strings (PYTHON_PREFIX, PYTHON_VERSION, PIP_PREFIX, ROS_VERSION) with the appropriate values according to your operating systems as follows:

| Ubuntu 18:                   | **Ubuntu 20**:               |
| ---------------------------- | ---------------------------- |
| PYTHON_PREFIX = python3      | PYTHON_PREFIX = python3      |
| PYTHON_VERSION = 3.5         | PYTHON_VERSION = 3.8         |
| ROBOTPKG_PYTHON_VERSION=py35 | ROBOTPKG_PYTHON_VERSION=py38 |
| PIP_PREFIX = pip3            | PIP_PREFIX = pip3            |
| ROS_VERSION = bionic         | ROS_VERSION = noetic         |

**NOTE:** ROS is no longer supported (only ROS2 Humble) on Ubuntu 22 hence is not possible to install Locosim on Ubuntu 22.

### Install ROS 

setup your source list:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys:

```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

install ROS main distro:

```
sudo apt-get install ros-ROS_VERSION-desktop-full
```

install ROS packages:

```
sudo apt-get install ros-ROS_VERSION-urdfdom-py
```

```
sudo apt-get install ros-ROS_VERSION-srdfdom
```

```
sudo apt-get install ros-ROS_VERSION-joint-state-publisher
```

```
sudo apt-get install ros-ROS_VERSION-joint-state-publisher-gui
```

```
sudo apt-get install ros-ROS_VERSION-joint-state-controller 
```

```
sudo apt-get install ros-ROS_VERSION-gazebo-msgs
```

```
sudo apt-get install ros-ROS_VERSION-control-toolbox
```

```
sudo apt-get install ros-ROS_VERSION-gazebo-ros
```

```
sudo apt-get install ros-ROS_VERSION-controller-manager
```

```
sudo apt install ros-ROS_VERSION-joint-trajectory-controller
```

### Pinocchio stuff

**Add robotpkg as source repository to apt:**

```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

**Register the authentication certificate of robotpkg:**

```
sudo apt install -qqy lsb-release gnupg2 curl
```

```
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
```

You need to run at least once apt update to fetch the package descriptions:

```
sudo apt-get update
```

Now you can install Pinocchio and the required libraries:

```
sudo apt install robotpkg-PINOCCHIO_PYTHON_VERSION-eigenpy	
```

```
sudo apt install robotpkg-PINOCCHIO_PYTHON_VERSION-pinocchio
```

```
sudo apt-get install robotpkg-PINOCCHIO_PYTHON_VERSION-quadprog  
```

**NOTE:** If you have issues in installing robotpkg libraries you can try to install them through ROS as:

```
sudo apt-get install ros-ROS_VERSION-LIBNAME
```



###  Python

```
sudo apt-get install PYTHON_PREFIX-scipy
```

```
sudo apt-get install PYTHON_PREFIX-matplotlib
```

```
sudo apt-get install PYTHON_PREFIX-termcolor
```

```
sudo apt install python3-pip
```

```
PIP_PREFIX install cvxpy==1.2.0
```

### **Support for Realsense camera (simulation)**

This packages are needed if you want to see the PointCloud published by a realsense camera attached at the endeffector. To activate it, you should load the xacro of the ur5 with the flag "vision_sensor:=true". 

```
sudo apt-get install ros-ROS_VERSION-openni2-launch
```

```
sudo apt-get install ros-ROS_VERSION-openni2-camera
```

```
sudo apt install ros-ROS_VERSION-realsense2-description
```

### **Support to simulate Grasping**

Unfortunately grasping in Gazebo is still an open issue, I impelented grasping using this [plugin]( https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation) that creates a fixed link between the gripper and the object to be grasped. To activate the grasping plugin set gripper_sim parameter to True in your configuration file. The following dependencies are required:

```
sudo apt-get install ros-ROS_VERSION-eigen-conversions 
```

```
sudo apt-get install ros-ROS_VERSION-object-recognition-msgs
```

```
sudo apt install ros-ROS_VERSION-roslint
```

You can check which parameters have to be tuned looking to the following [wiki]( https://github-wiki-see.page/m/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) 



# Configure environment variables 

```
gedit  ~/.bashrc
```

copy the following lines (at the end of the .bashrc), remember to replace the string PYTHON_VERSION with the appropriate version name as explained in [software versions](#software-versions) section:

```
source /opt/ros/ROS_VERSION/setup.bash
source $HOME/ros_ws/install/setup.bash
export PATH=/opt/openrobots/bin:$PATH
export LOCOSIM_DIR=$HOME/ros_ws/src/locosim
export PYTHONPATH=/opt/openrobots/lib/pythonPYTHON_VERSION/site-packages:$LOCOSIM_DIR/robot_control:$PYTHONPATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/openrobots/share/
```

the .bashrc is a file that is **automatically** sourced whenever you open a new terminal.

**NOTE**: people with some operating systems like ARCH LINUX, might need to add "export ROS_IP=127.0.0.1" to the .bashrc.




# Download code and setup ROS workspace

Now that you installed all the dependencies you are ready to get the code, but first you need to create a ros workspace to out the code in:

```
mkdir -p ~/ros_ws/src
```

```
cd ~/ros_ws/src
```

Now you can clone the repository inside the ROS workspace you just created:

```
git clone https://github.com/mfocchi/locosim.git --recursive
```

this command should also clone all the submodules, if you want to update them separately just run

```
git submodule update --init --recursive
```

source the environment variables

```
source $HOME/.bashrc
```

now compile (then this step won't bee needed anymore if you just work in python unless you do not modify / create additional ROS packages)

```
cd ~/ros_ws/
```

```
catkin_make install
```

the install step install the ros packages inside the "$HOME/ros_ws/install" folder rather than the devel folder. This folder will be added to the ROS_PACKAGE_PATH instead of the devel one. Whenever you modify some of the ROS packages (e.g. the ones that contain the xacro fles inside the robot_description folder), you need to install them to be sure they are been updated in the ROS install folder. 

Finally, run (you should do it any time you add a new ros package)

```
rospack profile
```

There are some additional utilities that I strongly suggest to install. You can find the list  [here](https://github.com/mfocchi/locosim/blob/develop/figs/utils.md).

**IMPORTANT!**

The first time you compile the code the install folder is not existing, therefore won't be added to the PYTHONPATH with the command **source $HOME/ros_ws/install/setup.bash**, and you won't be able to import the package ros_impedance_controller. Therefore, **only once**, after the first time that you compile, run again :

```
source $HOME/ros_ws/install/setup.bash
```

Now you are ready to run the code as explained  [here](https://github.com/idra-lab/locosim?tab=readme-ov-file#running-the-software-from-python-ide-pycharm).



Installing Git and SSH key (optional)
==============

To install Git you can open a terminal and run:

```
sudo apt install git
```

After that, you can check the version installed and configure the credentials with:

```
$ git --version
$ git config --global user.name <"name surnace">
$ git config --global user.email <"youremail@yourdomain.it">
```

After this, if you don't have an SSH key for your Github account, you need to create a new one to use the repositories:

* go to Settings/SSH  and GPG Keys

* open a terminal and run these commands:

  ```
  $ ssh-keygen 
  $ cd ~/.ssh/
  $ cat id_rsa.pub
  ```

  Copy the content of your public SSH into the box at the link before and press "New SSH key". You can now clone the  repositories with SSH without having to issue the password every time (I suggest to do not set any passphrase).



# Tips and Tricks  (optional)

1) Most of virtual machines including Virtualbox, do not have support for GPU. This means that if you run Gazebo Graphical User Interface (GUI) it can become very **slow**. A way to mitigate this is to avoid to start the  Gazebo GUI and only start the gzserver process that will compute the dynamics, you will keep the visualization in Rviz. This is referred to planners that employ BaseController or BaseControllerFixed classes. In the Python code where you start the simulator you need to pass this additional argument as follows:

```
additional_args = 'gui:=false'
p.startSimulator(..., additional_args =additional_args)
```

2) Another annoying point is the default timeout to kill Gazebo that is by default very long. You can change it (e.g. to 0.1s) by setting the  _TIMEOUT_SIGINT = 0.1 and _TIMEOUT_SIGTERM = 0.1:

```
sudo gedit /opt/ros/ROS_VERSION/lib/PYTHON_PREFIX/dist-packages/roslaunch/nodeprocess.py
```

 this will cause ROS to send a `kill` signal much sooner.



