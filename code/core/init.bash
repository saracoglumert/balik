## Variables ##
NEWHOSTNAME="robot"

## Configure OS ##
# Update and Upgrade
sudo apt update && sudo apt upgrade

#Change Hostname
sudo rm /etc/hostname
touch /etc/hostname
echo $HOSTNAME > /etc/hostname

#Install SSH Server
sudo apt install openssh-server -y

# Install Avahi
sudo apt install avahi-daemon -y

# Install Desktop Environment
sudo apt install ubuntu-Desktop -y

#Install Remote Desktop Server
sudo apt install xrdp -y

## Install ROS2
# Add PGP Key
sudo apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc

# Add Repo
sudo apt-add-repository http://packages.ros.org/ros2/ubuntu

# Install ROS2
sudo apt install ros-foxy-ros-base