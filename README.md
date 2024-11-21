# Convertible UAV-to-UGV Project

## 📋 Project Overview
Innovative BTech project developing a transformable unmanned aerial vehicle (UAV) that converts to an unmanned ground vehicle (UGV), tested and developed in Ubuntu 22.04. (this repo contains the packages for the simulation)

## 🛠 System Requirements
- **Operating System**: Ubuntu 22.04
- **ROS Version**: ROS2 Humble Distribution
- **Simulator**: Gazebo Classic 11

## 📦 Software Dependencies
### ROS2 Installation
go through this doc for the basic installation of [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
Note: During the installation of ROS2 Humble for the full basic installtion use the command given below instead of the default one for the installation
```bash
sudo apt install ros-humble-desktop-full
```
### Gazebo-Classic Installation
Go through this doc for the installation of Gazebo-Classic and its ros based dependencies ----> [Gazebo-Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
Note: to install the ros dependencies, sometimes you may have to use the command below just in case some of the required plugins aren't installed
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

