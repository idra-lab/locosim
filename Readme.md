

# What is Locosim?

Locosim is a didactic framework to learn/test basic controllers schemes on quadruped robots (HyQ/Solo/Aliengo/Go1 are supported) and manipulators (UR5  is supported). Locosim has been successfully tested on Ur5, Aliengo and Go1 robots and (to my knowledge) is the first Open-Source Easy-to-Use interface to the new Unitree Quadruped robot Go1. If you just bought a Go1 robot and you want to give a try, follow this [wiki](https://github.com/mfocchi/locosim/blob/develop/figs/go1_setup.md)!

Locosim is composed by a **roscontrol** node called **ros_impedance_controller** (written in C++) that interfaces a python ROS node (where the controller is written) to a Gazebo simulator. All the didactic labs have a description, with exercises of increasing complexity, in the folder **lab_descriptions** inside robot_control submodule. For each controller, plotting / logging utilities are available to evaluate the results together with a configuration file (LX_conf.py) to change the controller parameters. 

You have 3 ways to get the Locosim code:   1) with docker 2) by manual installation of dependencies.

**Note**: If you intend to use Locosim for your *research* please cite:

- M. Focchi, F. Roscia, C. Semini, **Locosim: an Open-Source Cross-Platform Robotics Framework**, Synergetic Cooperation between Robots and Humans. CLAWAR, 2023.  

  you can download a pre-print of the paper [here](https://iit-dlslab.github.io/papers/focchi23clawar.pdf). [View BibTeX](https://github.com/mfocchi/locosim/blob/develop/locosim.bib)

 

# Docker Installation

You can alternatively use a docker image that contains Ubuntu 20 and all the required dependencies already installed (you will need only to clone the code and compile it).

**LINUX**: follow this  [procedure](https://github.com/idra-lab/locosim/blob/master/install_docker_linux.md).

**MAC:** follow this  [procedure](https://github.com/idra-lab/locosim/blob/master/install_docker_mac.md).

**WINDOWS:** follow this [procedure](https://github.com/idra-lab/locosim/blob/master/install_docker_windows.md).



# Native Installation 

**LINUX:** follow this [procedure](https://github.com/idra-lab/locosim/blob/master/install_native.md).

**MAC:** follow this [procedure](https://github.com/idra-lab/locosim/blob/master/install_native.md), just replace **"sudo apt install package_name"** with **"brew install package_name"**.

**WINDOWS:** Install Ubuntu 20.4.06 LTS  following this procedure: https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overview

If you experiment any issue in using the Nvidia with  OpenGL rendering (the symptom is that you cannot visualize STL meshes in RVIZ) then you should update to the latest mesa-driver:

```
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt update
sudo apt install mesa-utils
```

then follow this [procedure](https://github.com/idra-lab/locosim/blob/master/install_native.md).



# **Running the software from Python IDE: Pycharm**

Now that you compiled the code you are ready to run the software! 

We recommend to use an IDE to run and edit the python files, like Pycharm community. 

1. To install it, enter in the $HOME folder of the docker and download it from here:

```
$ wget https://download.jetbrains.com/python/pycharm-community-2021.1.1.tar.gz
```

2. Then, unzip the program:

```
$tar -xf pycharm-community-2021.1.1.tar.gz
```

 and unzip it  *inside* the docker (e.g. copy it inside the `~/trento_lab_home` folder. 

**IMPORTANT**!** I ask you to download this specific version (2021.1.1) that I am sure it works, because the newer ones seem to be failing to load environment variables! 

3. To run Pycharm community type (if you are lazy you can create an alias...): 

```
$ pycharm-community-2021.1.1/bin/pycharm.sh
```

Running pycharm from the terminal enables to use the environment variables loaded inside the .bashrc.

4. click "Open File or Project" and open the folder robot_control. Then launch one of the labs in locosim/robot_control/lab_exercises or in locosim/robot_control/base_controllers  (e.g. ur5_generic.py)  right click on the code and selecting "Run File in Pyhton Console"

5. the first time you run the code you will be suggested to select the appropriate interpreter (/usr/binpython3.8). Following this procedure you will be sure that the run setting will be stored, next time that you start Pycharm.



# Running the Software from terminal

To run from a terminal we  use the interactive option that allows  when you close the program have access to variables:

```
$ python3 -i $LOCOSIM_DIR/robot_control/base_controllers/base_controller.py
```

to exit from python3 console type CTRL+Z



Installing NVIDIA drivers (optional)
==============

If your PC is provided with an NVIDIA graphics card, you can install its drivers in Ubuntu by following these steps:

add the repository

```
sudo add-apt-repository ppa:graphics-drivers/ppa
```

update the repository list:

```
sudo apt-get update
```

Install the driver, note that for Ubuntu 20.04 the 515 version is ok, for Ubuntu 22.04 the 535 is ok, but you can use also other versions:

```
sudo apt-get install nvidia-driver-X
```

The reboot the system

```
sudo reboot
```

Now tell the system to use that driver:

* open the _Software & Updates_ application
* go to "Additional Drivers" and select the latest driver you just installed with "proprietary, tested" description
* press on "Apply Changes".

You can verify if the drivers are installed by opening a terminal and running:

```
nvidia-smi
```

If this does not work, and you are sure you correctly installed the drivers, you might need to deactivate the "safe boot" feature from your BIOS, that usually prevents to load the driver. 



# Using the real robots

These packages are needed if you are willing to do experiments with the **real** robots

### **Universal Robot UR5**

The driver for the UR5 has already been included in Locosim but is not compiled by default, hence you need to:

1. remove file [CATKIN_IGNORE](https://github.com/mfocchi/universal_robots_ros_driver/blob/master/CATKIN_IGNORE) inside the **ur_driver** package 
2. remove file [CATKIN_IGNORE]( https://github.com/mfocchi/zed_wrapper/blob/af3750a31c1933d4f25b0cb9d5fc4de657d62001/CATKIN_IGNORE) inside the **zed_wrapper** package 
3. Install these additional packages:

```
sudo apt install ros-ROS_VERSION-ur-msgs
```

```
sudo apt install ros-ROS_VERSION-speed-scaling-interface
```

```
sudo apt install ros-ROS_VERSION-scaled-joint-trajectory-controller
```

```
sudo apt install ros-ROS_VERSION-industrial-robots-status-interface
```

```
sudo apt install ros-ROS_VERSION-speed-scaling-state-controller
```

```
sudo apt install ros-ROS_VERSION-ur-client-library
```

```
sudo apt install ros-ROS_VERSION-pass-through-controllers
```

4. recompile with **catkin_make install**.
5. add the following alias to your .bashrc

```
launch_robot='roslaunch ur_robot_driver ur5e_bringup.launch headless_mode:=true robot_ip:=192.168.0.100 kinematics_config:=$LOCOSIM_DIR/robot_hardware_interfaces/ur_driver/calibration_files/my_robot_calibration_X.yaml'
```

where X is {1,2}. For the robot with the soft gripper X = 2. 

4. Trigger the robot workbench power switch on  and press the on button on the UR5 Teach Pendant

5. Connect the Ethernet cable to the lab laptop and create a local LAN network where you set the IP of your machine to 192.168.0.101 (the robot IP will be 192.168.0.100, double check it using the UR5 "Teach Pendant"), see [network settings](https://github.com/mfocchi/locosim/blob/develop/network_settings.md) to setup the network in order to run together with a mobile platform.

6. Verify that you can ping the robot:

   ```
   ping 192.168.0.100
   ```

7. Where it says "Spegnimento" activate the robot pressing "Avvio" twice until you see all the 5 green lights and you hear the release of the brakes. Set the "Remote Control" in the upper right corner of the "Teach Pendant"

8. Run the **launch_robot** alias to start the **ur_driver**. If you want to start the driver without the ZED camera (e.g. you do not have CUDA installed), append **vision_sensor:=false** to the command. 

   ```
   launch_robot vision_sensor:=false
   ```

   Conversely, if you want to launch only the ZED camera alone and see the data in rviz:

   ```
   roslaunch zed_wrapper zed2.launch rviz:=true
   ```

9. Run the **ur5_generic.py** with the  [real_robot](https://github.com/mfocchi/robot_control/blob/2e88a9a1cc8b09753fa18e7ac936514dc1d27b8d/lab_exercises/lab_palopoli/params.py#L30) flag set to **True**. The robot will move to the home configuration defined  [here](https://github.com/mfocchi/robot_control/blob/babb5ff9ad09fec32c7ceaeef3d02715c6d067ab/lab_exercises/lab_palopoli/params.py#L26).

10. For the usage and calibration of the ZED camera follow this instructions [here](https://github.com/mfocchi/locosim/blob/develop/figs/zed_calibration.md).



**Universal Robots  + Gripper**

You have 2 kind of gripper available in the ur5 robot: a rigid 3 finger gripper and a soft 2 finger gripper.  Locosim seamlessly allows you to deal with both of them. By default, the 3 finger gripper is enabled, if you want to use the 2 finger one (soft_gripper)  you need to:

1. set the  [soft_gripper](https://github.com/mfocchi/robot_control/blob/fde3b27884819e1b2ea319fe5b2781a86d33a648/lab_exercises/lab_palopoli/params.py#L33) flag to True (in Simulation)

2. append **soft_gripper:=true** to the  **launch_robot** alias (on the real robot).

   

**Universal Robots  + ZED camera support**

If you are willing to use the real ZED Camera on the real robot, the zed_wrapper package is not compiled by default, hence you need to:

1. remove the file [CATKIN_IGNORE]( https://github.com/mfocchi/zed_wrapper/blob/af3750a31c1933d4f25b0cb9d5fc4de657d62001/CATKIN_IGNORE)  in the **zed_wrapper** package 

2. Install CUDA (locosim is compatible only with version 12): https://developer.nvidia.com/cuda-downloads

3. Install the  SDK library (locosim is compatible only with version 4.0.2) in: https://www.stereolabs.com/developers/release/

4. Recompile with catkin_make install

If you have issues remove the build/devel folder and recompile.



### Go1 Quadruped Robot

install these packages:

```
sudo apt-get install apt-get install liblcms2-2
```

```
sudo apt-get install apt-get install liblcms-bin
```

and follow this [wiki](https://github.com/mfocchi/locosim/blob/develop/go1_setup.md)!

