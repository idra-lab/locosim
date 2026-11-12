 

Install Docker on Windows 
================================================================================

1. First Install  Windows Subsystem for Linux (WSL). Open a command prompt with **administration** privileges and type. 

``` powershell
wsl --install
```

2. install Ubuntu 20.04.06 LTS from Microsoft Store. All the procedure is explained in detail here:

   https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overv

3. From Start run  Ubuntu 20.04.06 LTS, this is the procedure you should use to open a new terminal (do not use WSL).

4. follow this  [procedure](https://github.com/idra-lab/locosim/blob/master/install_docker_linux.md)



## **Code Management:**

To create your own code install the tool Visual Studio Code application together with the WSL extension as explained [here](https://code.visualstudio.com/docs/remote/wsl ). This will enable you to edit the code contained in the **trento_lab_home/ros_ws/src** folder.  

To be able to open more terminals on the same window install **terminator**:

```powershell
$ sudo apt install terminator
```



## **Nvidia Support**:

If you have a Nvidia GPU install the driver assiciated to your GPU model **directly** in windows downloading it from this [link]( https://www.nvidia.com/Download/index.aspx?lang=en-us ) (not inside Ubuntu!)

You can check if everything works with:

```
nvidia-smi
```

If you experiment any issue in using the Nvidia with OpenGL rendering (the symptom is that you cannot visualize STL meshes in RVIZ) then you should update to he latest mesa-driver:

```
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt update
sudo apt install mesa-utils
```



