#!/bin/bash
#Author: Loan MICHAUD
#Date: 11/10/2023
#Objective : install all packages for ROS2, GAZEBO and Turtlebot3 Simulation

#Check updates and upgrades
echo "CHECK UPDATES AND UPGRADES"
sudo apt update
sudo apt upgrade

#Requires installation before 
echo  "INSTALL PYTHON3 COLCON"
sudo apt install python3-colcon-common-extensions
sudo apt install curl

#ROS 2 Humble Installation packages :
#1.Add the ROS 2 apt repository to your system: 
echo "HUMBLE INSTALLATION PACKAGES"
echo "ADD THE ROS2 APT REPOSITORY TO YOUR SYSTEM"
sudo apt install software-properties-common
sudo add-apt-repository universe

#2.Add the ROS 2 GPG key with apt
echo "ADD THE ROS 2 GPG KEY WITH APT"
sudo apt update && sudo apt install curl -y 
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#3.Add the repository to your sources list:
echo "ADD THE REPOSITORY TO YOUR SOURCE LIST"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#4.Install ROS 2 packages (ROS, Rviz, Demos, Tutorials):
echo "INSTALL ROS 2 PACKAGES (ROS, Rviz, Demos, Turorials)"
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

#5.ROS-Base install: Communication libraries, message packages, command line tools (No GUI tools):
echo "ROS-BASE INSTALL : COMMUNICATION LIBRAIRIES, MESSAGE PACKAGES, COMMAND LINE TOOLS (NO GUI TOOLS)"
sudo apt install ros-humble-ros-base

#6.Development tools: Compilers and other tools to build ROS packages
echo "DEVELOPMENT TOOLS : COMPILERS AND OTHERS TOOLS TO BUILD ROS PACKAGES"
sudo apt install ros-dev-tools

#7.Environment setup
echo "ENVIRONMENT SETUP"
source /opt/ros/humble/setup.bash


#Gazebo Garden Installation packages:
echo "GAZEBO INSTALLATION PACKAGES"
#1.Install some necessary tools:
echo "INSTALL SOME NECESSARY TOOLS"
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

#2.Install Gazebo
echo "INSTALL GAZEBO"
sudo apt install gazebo
sudo apt install libgazebo-dev


#Turtlebot3 Humble Installation packages:
echo "TURTLEBOT3 HUMBLE INSTALLATION PACKAGES"
#1.Install Simulation package:
echo "INSTALL SIMULATION PACKAGE"
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

sudo apt remove ros-humble-turtlebot3-msgs
sudo apt remove ros-humble-turtlebot3
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
colcon build
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc

cd ~/turtlebot3_ws/src/
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build

#Others packages installation:
#Install Nav2 and SLAM Toolbox
echo "CHECK OR INSTALL NAV2 AND SLAM TOOLBOX"
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
