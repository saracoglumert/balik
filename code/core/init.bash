## Configure OS ##
#Install SSH Server
sudo apt install openssh-server

# Install Avahi
sudo apt install avahi-daemon

## Install ROS2
# Add PGP Key
sudo apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc

# Add Repo
sudo apt-add-repository http://packages.ros.org/ros2/ubuntu

# Install ROS2
sudo apt install ros-foxy-ros-base