# ==========================================================
# Ubuntu 20.04 + ROS Noetic full desktop
# ==========================================================
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# -----------------------------
# Base system dependencies
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales tzdata curl wget gnupg2 lsb-release ca-certificates \
    software-properties-common git build-essential cmake \
    python3 python3-pip python3-dev \
    libatlas-base-dev gfortran libgmp-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

# -----------------------------
# Install ROS build tools
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep python3-vcstool  \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Initialize rosdep
# -----------------------------
RUN rosdep init || true && rosdep update

# -----------------------------
# Install useful ROS packages
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-urdfdom-py \
    ros-noetic-srdfdom \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-joint-state-controller \
    ros-noetic-gazebo-msgs \
    ros-noetic-control-toolbox \
    ros-noetic-gazebo-ros \
    ros-noetic-controller-manager \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-catkin \
    ros-noetic-openni2-launch \
    ros-noetic-openni2-camera \
    ros-noetic-realsense2-description \
    ros-noetic-eigen-conversions \
    ros-noetic-object-recognition-msgs \
    ros-noetic-roslint \
    ros-noetic-ur-msgs \
    ros-noetic-speed-scaling-interface \
    ros-noetic-scaled-joint-trajectory-controller \
    ros-noetic-speed-scaling-state-controller \
    ros-noetic-ur-client-library \
    ros-noetic-pass-through-controllers \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-transmission-interface \
    ros-noetic-imu-tools \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-tools \
    ros-noetic-industrial-robot-status-controller \
    rospack-tools \
    liburdfdom-tools \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Python 
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-scipy \
    python3-matplotlib \
    python3-termcolor \
    python3-tk \
    python3-catkin-pkg \
    python3-rospkg \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Python wheels / packages (separate RUNs to avoid passing shell text to pip)

# upgrade pip/build tools first so pyproject.toml is parsed correctly
RUN pip3 install --upgrade pip setuptools wheel

# keep preinstalled numerical wheels to speed builds and avoid unnecessary compilation
RUN pip3 install --no-cache-dir numpy scipy wheel

# then install cvxpy and the other packages
RUN pip3 install --no-cache-dir cvxpy==1.2.0 shapely quadprog

# -----------------------------
# robotpkg (Focal → py38)
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-eigenpy \
    ros-noetic-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Utilities and GUI dependencies
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    gedit gitg git-cola iputils-ping net-tools \
    libxext6 libxrender1 libxtst6 libxi6 libfreetype6 fonts-dejavu \
    libgl1-mesa-glx libgtk-3-0 \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Patch roslaunch to reduce shutdown timeout
# -----------------------------
RUN python3 - <<'PY'
import re, pathlib
paths = [
    pathlib.Path("/opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py"),
    pathlib.Path("/usr/lib/python3/dist-packages/roslaunch/nodeprocess.py"),
]
for p in paths:
    if p.exists():
        s = p.read_text()
        s = re.sub(r'(_TIMEOUT_SIGINT\s*=\s*)[0-9.]+', r'\g<1>0.1', s)
        s = re.sub(r'(_TIMEOUT_SIGTERM\s*=\s*)[0-9.]+', r'\g<1>0.1', s)
        p.write_text(s)
        print("Patched:", p)
        break
PY

# Source ROS automatically  use single quote to have it literal and avoid expansion of empty $PYTHONPATH
RUN echo '#!/bin/bash' >> /etc/bash.bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash' >> /etc/bash.bashrc && \
    echo 'export LOCOSIM_DIR=/root/ros_ws/src/locosim' >> /etc/bash.bashrc && \
    echo 'export DISABLE_ROS1_EOL_WARNINGS=1' >> /etc/bash.bashrc && \
    echo 'export PYTHONPATH=/opt/ros/noetic/lib/python3.8/python3.8/site-packages:$LOCOSIM_DIR/robot_control:$PYTHONPATH' >> /etc/bash.bashrc && \
    echo 'alias pycharm='\''/root/pycharm/bin/pycharm.sh'\''' >> /etc/bash.bashrc
RUN chmod +x /etc/bash.bashrc

# your container now always runs Bash as a login shell, /etc/bash.bashrc will be sourced automatically.
ENTRYPOINT []
# Start bash by default (interactive login shell)
CMD ["/bin/bash", "--login", "-i"]

