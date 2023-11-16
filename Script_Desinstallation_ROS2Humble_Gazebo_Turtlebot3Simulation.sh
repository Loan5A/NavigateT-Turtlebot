#!/bin/bash
#Author: Loan MICHAUD
#Date: 10/10/2023

#Uninstall ROS2 Humble
sudo apt remove ~nros-humble-* && sudo apt autoremove
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
sudo apt upgrade

#Uninstall Gazebo 
sudo apt-get remove gazebo
sudo apt-get -y autoremove gazebo
sudo apt-get -y purge gazebo
sudo apt-get -y autoremove

