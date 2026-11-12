Installation with Docker
================================================================================

- Run the script install_docker.sh. This script is important because it installs the docker client on your machine and adds to your user the privileges to run the docker images, and create new folder.


```
$sudo apt install curl
$curl -o install_docker.sh https://raw.githubusercontent.com/idra-lab/locosim/refs/heads/master/docker/install_docker.sh
$sudo chmod +x install_docker.sh
$./install_docker.sh
```
- If everything went smooth you should read: **To start docker, reboot the system!** You can now restart the PC so that all changes made can be applied.
- If you look into your **host** Ubuntu home directory, you will see that the **trento_lab_home** directory has been created with **/ros_ws/src** subfolders.
- now you can clone the loco_nav code inside the  **trento_lab_home/ros_ws/src** folder


```
$ cd ~/trento_lab_home/ros_ws/src
$ git clone https://github.com/idra-lab/locosim.git --recursive
```

**NOTE:**  when you clone the code, be sure to have a stable and fast connection. Before continuing, be sure you properly checked out **all** the submodules without any error.

- Now you have two options: 

  A) Download the docker image from here:	

  ```
  docker pull mfocchi/trento_lab_framework:locosim 
  ```

  B) compile the docker image yourself:

  ```
  cd ~/trento_lab_home/ros_ws/src/locosim/docker
  docker build -t mfocchi/trento_lab_framework:locosim -f Dockerfile .
  ```

- Now, you need to configure the bash environment of your Ubuntu machine as follows. Open the `bashrc` file from your home folder:


```
$ gedit ~/.bashrc
```

- and add the following lines at the bottom of the file:

```bash
alias lab_locosim='xhost +local:root; \
docker rm -f docker_container >/dev/null 2>&1 || true; \
docker run --name docker_container --gpus all \
--workdir="/root" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--device=/dev/dri:/dev/dri \
--network=host --hostname=docker -it \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--privileged --shm-size 2g --rm \
--volume $HOME/trento_lab_home:/root \
mfocchi/trento_lab_framework:locosim'
alias dock-other='docker exec -it docker_container /bin/bash'
```

- Load the .bashrc script (next time you will open a terminal this will be automatically loaded).


```
$ source ~/.bashrc
```

**NOTE!** If you do not have an Nvidia card in your computer, you should skip the parts about the installation of the drivers, and you can still run the docker **without** the **--gpus all**  in the **lab_locosim** alias.

- Open a terminal and run the "lab_locosim" alias:

```
$ lab_locosim
```

- You should see your terminal change from `user@hostname` to `user@docker`. 
- the **lab_locosim** script will mount the folder `~/trento_lab_home` on your **host** computer. Inside of all of the docker images this folder is mapped to `$HOME`.This means that any files you place   in your docker $HOME folder will survive the stop/starting of a new docker container. All other files and installed programs will disappear on the next run. 
- The alias **lab_locosim** needs to be called only ONCE and opens the image. To link other terminals to the same image you should run **dock-other**, this second command will "**attach**" to the image opened previously by calling the **lab_locosim** alias.  You can call **lab_locosim** only once and **dock-other** as many times you need to open multiple terminals.



# Compiling the code

- Now you can compile the ROS workspace in the $HOME directory **inside** docker:


```
$ cd  /root/ros_ws/
$ catkin_make install
```

- Only once, after the first compilation do:	

```
$ source /root/ros_ws/install/setup.bash
```

**NOTE:**  when you run the code, if an error pops-up that tells you a recently compiled package cannot be found you need to run:

```
$ rospack profile
```

This function crawls through the packages in ROS_ROOT  and ROS_PACKAGE_PATH, read and parse  the package.xml for each package, and  assemble a complete dependency tree  for all packages.

Now you are ready to run the code as explained  [here](https://github.com/idra-lab/locosim?tab=readme-ov-file#running-the-software-from-python-ide-pycharm).

When you have finished exit from the container typing:  

```
$ exit
```



# **Committing the image** (optional)

To install new packages open a terminal and call the apt install **without** sudo. To store the changes in the local image, get the ASH (a number) of the active container with:

```powershell
$ docker ps 
```

14. Commit the docker image (next time you will open an new container it will retain the changes done to the image without loosing them):

```powershell
$ docker commit ASH mfocchi/trento_lab_framework:locosim
```



# Docker Issues (optional)

<a name="docker_issues"></a>

Check this section only if you had any issues in running the docker!

- When launching any graphical interface inside docker (e.g. pycharm or gedit) you get this error:

```
No protocol specified
Unable to init server: Could not connect: Connection refused

(gedit:97): Gtk-WARNING **: 08:21:29.767: cannot open display: :0.0
```

It means that docker is not copying properly the value of you DISPLAY environment variable, you could try to solve it in this way, in a terminal **outside docker** launch:

```
echo $DISPLAY
```

and you will obtain a **value**  (e.g. :0) if you run the same command in a docker terminal the value will be different, then in the .bashrc inside the docker add the following line:

```
export DISPLAY=value
```

- If you still have issues because of the x11 library maybe you are running on wayland, be sure you have installed the qt5 library:

  ```
  apt install -y qt5-default
  ```

  and add this to your .bashrc

  ```
  export XDG_SESSION_TYPE=x11
  ```

  

  
