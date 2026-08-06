# ==============================================================================
# 1. SYSTEM PREPARATION & UPDATES
# ==============================================================================

# Update local package lists and upgrade existing packages
sudo apt update && sudo apt upgrade -y

# Install essential helper utilities for keys, certificates, and software management
sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates

# Enable the Ubuntu Universe repository (contains community dependencies for ROS 2)
sudo add-apt-repository universe

# Fix dependency indices by enabling noble-updates repository
sudo sed -i 's/Suites: noble/Suites: noble noble-updates noble-security/' /etc/apt/sources.list.d/ubuntu.sources

# Run a full distribution upgrade to update system libraries
sudo apt update
sudo apt dist-upgrade -y

# Reboot the system to apply core kernel updates
sudo reboot


# ==============================================================================
# 2. ADD ROS 2 REPOSITORIES & GPG KEY
# ==============================================================================

# Download the official ROS 2 GPG security key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository to your APT package sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# ==============================================================================
# 3. INSTALL ROS 2 JAZZY & CORE TOOLS
# ==============================================================================

# Refresh package lists to index the newly added ROS 2 repository
sudo apt update

# Install ROS 2 Base, Colcon build tool, and Rosdep dependency manager
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep

# Initialize and update rosdep database
sudo rosdep init
rosdep update


# ==============================================================================
# 4. ENVIRONMENT & NETWORK CONFIGURATION
# ==============================================================================

# Automatically load ROS 2 commands on every new terminal session
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Set matching Domain ID for laptop/Pi Wi-Fi communication
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Load the updated ~/.bashrc environment into the current terminal session
source ~/.bashrc


# ==============================================================================
# 5. CREATE ROS 2 WORKSPACE & VERIFY
# ==============================================================================

# Create workspace directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the empty workspace
colcon build

# Add the workspace overlay to your ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify ROS 2 installation
ros2 --help