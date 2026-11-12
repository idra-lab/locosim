# Index 
- [Index](#index)
- [Installation](#installation)
  - [Installing Docker Desktop for MacOS](#installing-docker-desktop-for-macos)
  - [Setting up the locosim folder](#setting-up-the-locosim-folder)
- [Running](#running)
  - [Pulling the Docker image](#pulling-the-docker-image)
  - [Running the Docker container](#running-the-docker-container)
  - [Editing files](#editing-files)
  - [Compiling and running the code](#compiling-and-running-the-code)
  - [Git](#git)
  - [Saving the image (optional)](#saving-the-image-optional)
- [WARNINGS](#warnings)
- [TL;DR](#tldr)


# Installation 

## Installing Docker Desktop for MacOS

While it is possible to install Docker for MacOS using [HomeBrew](https://brew.sh/), we suggest to use the official Docker installation that can be found [here](https://docs.docker.com/desktop/install/mac-install/).

Once you have installed Docker, you will find the Docker Desktop app in your Applications folder. Open it and follow the instructions to complete the setup.

To check that docker is running properly, open a terminal and type:
```bash 
docker run --rm hello-world
```

If this returns a message similar to the following, then you have not started Docker Desktop:
```
docker: Cannot connect to the Docker daemon at unix:///Users/user/.docker/run/docker.sock. Is the docker daemon running?.
```

Simply start Docker Desktop from the Applications folder and try again.

If you have problems with Docker installation, [contact me](mailto:enrico.saccon@unitn.it).


## Setting up the locosim folder

In your host machine, create the folder `locosim/ros_ws/src` in your home directory:

```bash
mkdir -p ~/locosim/ros_ws/src
```

Then navigate to it and clone the locosim repository with:
```bash
cd ~/locosim/ros_ws/src
git clone https://github.com/idra-lab/locosim.git
git submodule update --init 
```

**NOTE:**  when you clone the code, be sure to have a stable and fast connection. Before continuing, be sure you properly checked out **all** the submodules without any error.

# Running 

## Pulling the Docker image

- Download ONE of the docker images:

```bash
docker pull chaff800/locosim:noetic
docker pull chaff800/locosim:noetic_pycharm
```

The second image has PyCharm already installed based on [Readme.md](README.md) instructions.

The images are based on [this image](https://hub.docker.com/r/tiryoh/ros-desktop-vnc).

Change its name so that it is easier to remember:

```bash
docker tag chaff800/locosim:noetic locosim:noetic
```

## Running the Docker container

Before running the following commands, check section **WARNINGS** at the end of this document.

To run the container, use the command:

```bash
docker run --name locosim_c --rm -v ~/locosim/ros_ws/:/home/ubuntu/ros_ws/ -p 6080:80 --shm-size=512m --platform linux/amd64 locosim:noetic
```

If everything went smooth you should see the following message on the terminal:

```bash
INFO Included extra file "/etc/supervisor/conf.d/supervisord.conf" during parsing
INFO Set uid to user 0 succeeded
INFO RPC interface 'supervisor' initialized
CRIT Server 'unix_http_server' running without any HTTP authentication checking
INFO supervisord started with pid 38
INFO spawned: 'novnc' with pid 40
INFO spawned: 'vnc' with pid 41
INFO success: novnc entered RUNNING state, process has stayed up for > than 1 seconds (startsecs)
INFO success: vnc entered RUNNING state, process has stayed up for > than 1 seconds (startsecs)
```

You can now open a web browser and type the address [`http://localhost:6080/vnc.html`](`http://localhost:6080/vnc.html`) to open the VNC window. The desktop of the container should appear.

If needed, the username of the user inside the container is `ubuntu` and the password is `ubuntu`. The user does have root privileges on the container, not on the host machine.

**Note**: there may be errors regarding permissions denied for files in .git folders. You can ignore them, they should not affect the execution of the code. 

## Editing files

You can edit the files of the `locosim` package using any text editor or IDE installed on your host Mac machine. All changes will be reflected inside the container because you mounted the folder `locosim` as a volume.

Alternatively, if you want to edit files directly inside the container, the Codium IDE is already installed. To open it, click on the Codium icon on the desktop of the container. Also vim is already installed. Finally, the image is based on Ubuntu 18.04, so you can install (almost) any other text editor you may need.

## Compiling and running the code

To compile the code, open a terminal inside the container (there is a terminal icon on the desktop) and type:

```bash
cd /home/ubuntu/ros_ws/
catking_make 
catkin_make install
source /home/ubuntu/ros_ws/install/setup.bash
```

All dependencies should be already installed in the image. If you need to install new packages, remember to commit the image after installing them (see section **Saving the image (optional)** below).

Obviously, when you run the container again, you need to use the new image name instead of `locosim:noetic` in the docker run command.

**Note**: since we have mounted the `locosim` folder as a volume, the compiled files will be stored on your host Mac machine in the same folder. This means that if you re-create the container, you do not need to re-compile the code again unless you delete the compiled files from your host machine. If you do not want to have the compiled files on your host machine, you can remove the ` -v` option from the docker run command above. In this case, the compiled files will be stored inside the container and will be lost when you delete the container (unless you commit the image each time before exiting).

## Git

Git is installed in the container, but the permissions from the `.git` folder inside your host machine are not transferred to the container. The easiest way is to use git directly from your host Mac machine whenever you want to commit changes to the code or push. 

While I highly discourage this step, if you want to use git inside the container, you need to set up your ssh keys and git configuration inside the container as well. Also, remove the `locosim/ros_ws/src/locosim` folder in your host machine and clone the repository again from inside the container, otherwise you will have permission problems.

## Saving the image (optional)

Let's say that you have installed new packages inside the container or have made changes to the container that you want to save. You can commit the changes to a new image with the command:

```bash
docker commit ros_noetic_c <name you want to give to the new image>
```
9. Follow the instructions in the main [README](README.md) file to run the code. 

Add `-m "your message"` to add a commit message.

Obviously, when you run the container again, you need to use the new image name instead of `locosim:noetic` in the docker run command.

You do not need to save the image if you have just made changes to the code inside the mounted volume, as these changes are automatically saved on your host machine.

The container suggests to log out before committing the image to avoid problems. It should not be strictly necessary, but if you want to be safe, just log out before committing.


# WARNINGS

- The option ` -v` in the docker run command means that you are mounting a volume and ANY change you do inside the container in the folder `/home/ubuntu/ros_ws/src/locosim` will be reflected on your host machine in the folder `locosim/ros_ws/src/locosim`. Be careful when deleting files or folders inside the container because they will be deleted also on your host machine.
- The option `--rm` means that when you exit from the container, it will be deleted. If you want to keep the container after exiting, remove this option from the docker run command. It goes without saying, that the volumes that you mount with the ` -v` option will be kept on your host machine in any case.
- The option `--shm-size=512m` is important because it increases the size of the shared memory of the container. Some ROS nodes may need a big amount of shared memory to work properly. If you do not set this option, some nodes may crash during their execution.
- The option `--platform linux/amd64` is important if you are using a Mac with M* chip. It forces the container to run in emulation mode for x86_64 architecture. This may slow down a bit the execution of the code but it is necessary because most of the docker images available are built for x86_64 architecture.


# TL;DR

1. Install Docker Desktop for MacOS from [here](https://docs.docker.com/desktop/install/mac-install/).
2. Create the folder `locosim/ros_ws/src` in your home directory.
3. Clone the [locosim](https://github.com/mfocchi/locosim) repository in the folder `locosim/ros_ws/src` with submodules.
4. Pull the docker image with `docker pull chaff800/locosim:noetic` and tag it as `locosim:noetic`.
5. Run the container with the command: 
```bash
docker run --name ros_noetic_c --rm -v ~/locosim/:/home/ubuntu/ros_ws/ -p 6080:80 --shm-size=512m --platform linux/amd64 locosim:noetic
```
6. Open a web browser and go to [`http://localhost:6080/vnc.html`](http://localhost:6080/vnc.html) to open the VNC window.
7. Edit the code using any text editor or IDE on your host machine (files are in `~/locosim/ros_ws/src/locosim`).
8. Open a terminal inside the container and compile the code with:
```bash
cd /home/ubuntu/ros_ws/
catkin_make 
catkin_make install
source /home/ubuntu/ros_ws/install/setup.bash
```
9. Follow the instructions in [here](https://github.com/idra-lab/locosim?tab=readme-ov-file#running-the-software-from-python-ide-pycharm) to run the code.

