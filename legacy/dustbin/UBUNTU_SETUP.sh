# --- UBUNTU SETUP SCRIPT (Run inside WSL) ---
# 1. Open Ubuntu Terminal
# 2. Paste the following:

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key & Repo
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/lib/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble & Gazebo
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-gazebo-ros-pkgs -y

# Setup Workspace
mkdir -p ~/dustbin_ws/src
cd ~/dustbin_ws/
colcon build
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/dustbin_ws/install/setup.bash" >> ~/.bashrc
