Installation with Docker
================================================================================

- Run the script install_docker.sh. This script is important because it installs the docker client on your machine and adds to your user the privileges to run the docker images

```
$ ./install_docker.sh
```
- If everything went smooth you should read: **To start docker, reboot the system!** You can now restart the PC so that all changes made can be applied.

- If you look into your **host** Ubuntu home directory, you will see that the **trento_lab_home** directory has been created.

- if you have troubles using **gedit** use other editors like  **vim** or **nano** in place of gedit

  -  Download the docker image from here. It will be slow.

  ```
  $ docker pull mfocchi/trento_lab_framework:introrob_matlab
  ```

  - Now, you need to configure the bash environment of your Ubuntu machine as follows. Open the `bashrc` file from your home folder:


  ```
  $ gedit ~/.bashrc
  ```

  -  and add the following lines at the bottom of the file:

  ```bash
  alias lab='docker rm -f docker_container || true; docker run --name docker_container --gpus all  --user $(id -u):$(id -g)  --workdir="/home/$USER" --volume="/etc/group:/etc/group:ro"   --volume="/etc/shadow:/etc/shadow:ro"  --volume="/etc/passwd:/etc/passwd:ro" --device=/dev/dri:/dev/dri  -e "QT_X11_NO_MITSHM=1" --network=host --hostname=docker -it  --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume $HOME/trento_lab_home:$HOME --env=HOME --env=USER  --privileged  -e SHELL --env="DISPLAY=$DISPLAY" --shm-size 2g --rm  --entrypoint /bin/bash mfocchi/trento_lab_framework:introrob_matlab'
  alias dock-other='docker exec -it docker_container /bin/bash'
  alias dock-root='xhost +local:docker; docker exec -it --user root docker_container /bin/bash'
  ```

   **NOTE!** If you do not have an Nvidia card in your computer, you should skip the parts about the installation of the drivers, and you can still run the docker **without** the **-nv** flag in the **lab** alias.

  - Open a terminal and run the "lab" alias:

  ```
  $ lab
  ```

  - You should see your terminal change from `user@hostname` to `user@docker`. 
  - (**NOTE!**: If you don't find your license in this path `cd /usr/local/MATLAB/R2023a/licenses`, check the section **Activate license for MATLAB**.)

  - the **lab** script will mount the folder `~/trento_lab_home` on your **host** computer. Inside of all of the docker images this folder is mapped to `$HOME`.This means that any files you place   in your docker $HOME folder will survive the stop/starting of a new docker container. All other files and installed programs will disappear on the next run. 
  - Copy your Matlab licence in the `~/trento_lab_home/matlab` folder.
  - The alias **lab** needs to be called only ONCE and opens the image. To link other terminals to the same image you should run **dock-other**, this second command will "**attach**" to the image opened previously by calling the **lab** alias.  You can call **lab** only once and **dock-other** as many times you need to open multiple terminals.

  **NOTE!** If you do not have an Nvidia card in your computer, you should skip the parts about the installation of the drivers, and you can still run the docker **without** the **--gpus all ** flag in the **lab** alias. 

  - Now you need to edit the .bashrc script (that was created by the install script) **inside** the docker

    ```
    $ gedit ~/.bashrc
    ```

    and add the following lines at the bottom of the file:

    ```bash
    source /opt/ros/noetic/setup.bash
    source $HOME/ros_ws/install/setup.bash
    export PATH=/opt/openrobots/bin:$PATH
    export LOCOSIM_DIR=$HOME/ros_ws/src/locosim
    export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$LOCOSIM_DIR/robot_control:$PYTHONPATH
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/openrobots/share/
    ```

  - Now you can setup the workspace in the $HOME directory **inside** docker:

  ```
  $ source /opt/ros/noetic/setup.bash
  $ mkdir -p ~/ros_ws/src
  $ cd ~/ros_ws/src
  $ git clone git@github.com:idra-lab/locosim.git  -b climbing_robot --recursive
  $ cd  ~/ros_ws/
  $ catkin_make install
  $ source .bashrc
  ```

Activate license for Matlab
--------------
If you have online matlab license, you should to create a matlab license file (.lic), follow these steps:

open a terminal and run docker with the `lab` alias, after that open another docker and run the `dock-root` alias, after that run the following command in root docker (or `dock-root`):

```
cd /usr/local/MATLAB/R2023a/bin
./activate_matlab.sh 
```
After that, add your credentials. If you have university or online access, you should create a temporary password by following this link: [MathWorks Account](https://it.mathworks.com/mwaccount/).

Then, follow these steps:

1. Go to **my_account** → **Security settings** → **Generated temporary code**.
2. Copy the temporary password and paste it into the password field.

Finally, follow the steps indicated in MATLAB's GUI to create the license. In the **login name** field, enter the name used in Docker.

Once created, it should appear in the following folder:

```
cd /usr/local/MATLAB/R2023a/licenses
```
You should find the license. Now, copy it into the **.matlab** folder.

```
cp license_docker_872052_R2023a.lic $HOME/.matlab/
```
and that it!

Installing NVIDIA drivers (optional)
--------------

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



## Docker Issues (optional)

--------------------------------------------------------------------------------

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

- When installing docker using ./installation_tools/install_docker.sh you may have a pip3 syntax error. 

You could try to solve it in this way:

```
curl https://bootstrap.pypa.io/pip/3.5/get-pip.py -o get-pip.py
python3 get-pip.py
rm get-pip.py
```

- If you do not have Nvidia drivers installed, then make sure you are not using the `-nv` option when launching `lab-docker.py`. You may get a message in the terminal that looks like this:

  ![nvidia_issue](uploads/cd09602de0f7edd1e0432359754f495c/nvidia_issue.jpeg)

  

- Nvidia error: could not select device driver “” with capabilities:

You can solve this way:

```
sudo apt install -y nvidia-docker2
sudo systemctl daemon-reload
sudo systemctl restart docker
```







