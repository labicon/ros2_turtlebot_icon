# How to Turtlebot4
Author: Hongrui Zhao (hongrui5@illinois.edu)
Date: Feb/11/2025


## Useful Resources
Turtlebot 4 official user manual: https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html#robot
ROS2 Humble user manual: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
OAK-D Lite camera documents: https://docs.luxonis.com/software/ros/depthai-ros/driver/



## Setup your PC
Create3 bot, Pi are installed with ROS2 humble.
Therefore, you also need ROS2 humble for your PC.
Based on my testing, ROS2 jazzy on PC won't work with ROS2 humble on pi/create3.

ROS2 humble only supports Ubuntu 22 (not 24) or Linux mint 21.
So make sure your PC has the right linux distribution installed. 

My procedure to setup my PC is:
*  Install [linux mint 21](https://linuxmint.com/download_all.php) following the [instruction](https://linuxmint-installation-guide.readthedocs.io/en/latest/)
* Install ROS2 humble following [this instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
* Install Turtlebot4 ROS2 package by `sudo apt update && sudo apt install ros-humble-turtlebot4-desktop`
* Set up discovery server following [this instruction](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html). I will cover this in details later.
* Set up your conda environment to run your codes.
* Install [vicon-bridge](https://github.com/dasc-lab/ros2-vicon-bridge) to get VICON mocap data.
* Install my [ros2_turtlebot_icon](https://github.com/HongruiZhao/ros2_turtlebot_icon) node to subscribe to images and VICON data and send them to your python codes using redis. I will cover this in details later.

### Discovery Server
Since we have two Turtlebots, we need to set up the discovery servers for them at the same time.
Your setup should be:
`ROS_DOMAIN_ID [0]:` 
`Enter the information for the first discovery server`
`Discovery Server ID [0]:` 
`Discovery Server IP: 192.168.50.49`
`Discovery Server Port [11811]: `
`Re-enter the last server (r), add another server (a), or done (d): a`
`Enter the information for the next discovery server`
`Discovery Server ID [0]: 1`
`Discovery Server IP: 192.168.50.20`
`Discovery Server Port [11811]: `
`Re-enter the last server (r), add another server (a), or done (d): d`
`Configuring:`
` ROS_DOMAIN_ID=0`
` ROS_DISCOVERY_SERVER="192.168.50.49:11811;192.168.50.20:11811;`

### ros2_turtlebot_icon




## How to SSH into Turtlebots
Both our Turtlebots are connected to "iconlab-5G" wifi using "discovery server" mode.
First connect your PC to the same wifi.

For Turtlebot "miriel", do: 
`ssh ubuntu@192.168.50.49`
For Turtlebot "oogway", do:
`ssh ubuntu@192.168.50.20`

It is recommended to use SSH FS VScode extension.
With SSH FS, you don't need to manually type in the commands every time, and it makes it easy to edit configuration files!
