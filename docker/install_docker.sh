#!/bin/bash

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
	echo "install for LINUX"
	sudo apt-get install -y curl

	curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
	sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
	distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
	   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
	   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
	sudo apt-get update
	sudo apt-get install -y docker-ce nvidia-docker2  build-essential cmake

	# Add user to docker's group
	sudo usermod -aG docker ${USER}
	
	if [ -n "$WSL_DISTRO_NAME" ]; then
    		echo "Running inside WSL"
		# Overwrite /etc/wsl.conf with the desired network section
		echo "[network]" | sudo tee -a /etc/wsl.conf > /dev/null
    		echo "generateHosts = false" | sudo tee -a /etc/wsl.conf > /dev/null
	else
    		echo "Not running inside WSL"
	fi
	

elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "=== Installing for macOS (Intel or Apple Silicon) ==="

    ARCH=$(uname -m)
    echo "Detected architecture: $ARCH"

    # Install Homebrew if missing
    if ! command -v brew &> /dev/null; then
        echo "Homebrew not found. Installing..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    else
        echo "Homebrew already installed."
    fi

    # Install Docker Desktop (auto-selects Intel/ARM build)
    if ! ls /Applications | grep -q "Docker.app"; then
        echo "Installing Docker Desktop..."
        brew install --cask docker
    else
        echo "Docker Desktop already installed."
    fi

    # Intel-specific notice
    if [[ "$ARCH" == "x86_64" ]]; then
        echo "Intel Mac detected — using x86_64 Docker Desktop build."
    fi

    # Apple Silicon notice
    if [[ "$ARCH" == "arm64" ]]; then
        echo "Apple Silicon detected — using ARM Docker Desktop build."
    fi

    echo "⚠️  IMPORTANT: You must open Docker.app manually once after installation."
    


fi

# insert/update hosts entry
ip_address="127.0.0.1"
host_name="docker"
docker_folder="trento_lab_home"
# find existing instances in the host file and save the line numbers
matches_in_hosts="$(grep -n $host_name /etc/hosts | cut -f1 -d:)"
host_entry="${ip_address} ${host_name}"

echo "Please enter your password if requested."

if [ ! -z "$matches_in_hosts" ]
then
    echo "Docker entry already existing in etc/hosts."
else
    echo "Adding new hosts entry."
    echo "$host_entry" | sudo tee -a /etc/hosts > /dev/null
fi



if [ -d "${HOME}/trento_lab_home" ]; then
  echo "Directory trento_lab_home exists."
else
    echo "Creating trento_lab_home dir."
    mkdir -p ${HOME}/trento_lab_home/ros_ws/src
fi

# Check if source .ssh exists
if [ -d "$HOME/.ssh" ]; then
    echo "Copying .ssh folder with user permissions"
    # Ensure destination exists
    mkdir -p "$HOME/trento_lab_home/.ssh"

    # Copy contents
    sudo cp -R "$HOME/.ssh/"* "$HOME/trento_lab_home/.ssh/"

    # Set ownership
    sudo chown -R $USER:$USER "$HOME/trento_lab_home/.ssh"
fi



echo -e "${COLOR_BOLD}To start docker, reboot the system!${COLOR_RESET}"
