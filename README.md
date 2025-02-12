# How to Turtlebot4
Author: Hongrui Zhao (hongrui5@illinois.edu)
Date: Feb/11/2025


## Useful Resources
Turtlebot 4 official user manual: https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html#robot  
ROS2 Humble user manual: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html  
OAK-D Lite camera documents: https://docs.luxonis.com/software/ros/depthai-ros/driver/  
Create 3 documents: https://iroboteducation.github.io/create3_docs/hw/face/  


## Power on Turtlebots
Center Button: held for seven seconds to enter storage mode, unpower everything (it will flash white few times).  
the only way to power it on is to place it on the dock.  

LED ring: 
- pulsing red: when battery < 10%
- spinning white => Robot is booting up. Wait for "happy sound" to play.
- half solid yellow (only LED on the bottom lit): wheel disabled
- Green: success connecting to Wi-Fi. Give it ~10 seconds, it will go back  to white
- solid red: error. it will automatically cycle power. it usually caused by CPU overloaded from processing too many topics. 
- While charging, the percentage of white ring = battery %. All white = 100% battery 
- While charging ,pulsing red: when battery < 10%

Best way to disconnect pi (you want to charge the bot and turn pi off): pi power cable is connected to a connector, which connects to create3. unplug that connector. there is a notch beneath it that you can pull it open


## Set up your PC
Create3 bot, Pi are installed with ROS2 humble.  
Therefore, you also need ROS2 humble for your PC.  
Based on my testing, ROS2 jazzy on PC won't work with ROS2 humble on pi/create3.  

ROS2 humble only supports Ubuntu 22 (not 24) or Linux mint 21.  
So make sure your PC has the right linux distribution installed.   

My procedure to set up my PC is:  
*  Install [linux mint 21](https://linuxmint.com/download_all.php) following the [instruction](https://linuxmint-installation-guide.readthedocs.io/en/latest/)
* Install ROS2 humble following [this instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
* Install Turtlebot4 ROS2 package by `sudo apt update && sudo apt install ros-humble-turtlebot4-desktop`
* Set up discovery server following "User PC Setup" in [this instruction](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html). I will cover this in details later.
* Set up your conda environment to run your codes.
* Install [vicon-bridge](https://github.com/dasc-lab/ros2-vicon-bridge) to get VICON mocap data.
* Install my [ros2_turtlebot_icon](https://github.com/labicon/ros2_turtlebot_icon) node to subscribe to images and VICON data and send them to your python codes using redis. I will cover this in details later.

Since you will need lots of terminals, I highly recommend using tmux.  
Install tmux by: `apt install tmux`.  
Open your default terminal, then type in `tmux` to create a new tmux session.  
`ctrl+b+%` for vertical panel split.  
`ctrl+b+"` for horizontal panel split.  
`ctrl+b+q` to show panel number, and type in the number to go to that panel.  
`ctrl+b ctrl+up` (can hold ctrl and do multiple up): increase panel size.  
`ctrl+b ctrl+down` decrease panel size.  
`tmux rename-session -t some_number your_name` rename the tmux session from `some_number` to `your_name`. The number can be found at the bottom left.  
`tmux detach` go back to your default terminal.  
`tmux attach -t your_name` go back to the existing session `your_name`.  
`tmux ls` show all existing tmux sessions.  

## Discovery Server
Download and run the setup script by:  
`wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty`.    
You will be prompted for a few settings.  
Since we have two Turtlebots, we need to set up the discovery servers for them at the same time.  
Your settings should be:  
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




## ros2_turtlebot_icon
This is a little ROS2 node I wrote that subscribes to Turtlebot's RGBD images as well as VICON data.  
The node visualizes the RGBD images.   
The node also uses [redis](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/), a fast in-memory data structure server, to share the images and VICON data with other python programs.   
Install redis by `sudo apt install redis`.  
Check the installation by `redis-server --version`.  
start a server: `sudo systemctl start redis-server.service`.  
stop a server: `sudo systemctl stop redis-server.service`.  
check server status: `sudo systemctl status redis-server.service`.  
enable server starts at boot: `sudo systemctl enable redis-server.service`.  
once Redis is running, you can test it by running `redis-cli`. and type in  `ping`  to test connection, you should get a `PONG` back.  

Once a redis server is running, we can use redis-py to write a python client to connect to the server.  
install: `pip install redis`.  
An example client code is provided in this repository at `./redis_example.py`.  

Now back to our ROS2 node.  
We use "colcon" to build our package. Install colcon by: `sudo apt install python3-colcon-common-extensions`.  
From the directory `turtlebot_icon_ws`, build all packages by running: `colcon build --symlink-install`.  
Then setup the environment (need to do this every time you close the terminal): `source install/setup.bash`.  
Run the node (for example, for turtlebot `miriel`): `ros2 turtlebot_icon sub_redis_node miriel`.  




## Check battery status 
For example, to check oogway's battery, do: `ros2 topic echo /oogway/battery_state`.  
Messages will take a while to show up.  



## Drive your Turtlebots
For example, if you want to drive `oogway`:  
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=oogway/cmd_vel`.  


## How to SSH into Turtlebots
Both our Turtlebots are connected to "iconlab-5G" wifi using "discovery server" mode.  
First connect your PC to the same wifi.  

For Turtlebot "miriel", do:   
`ssh ubuntu@192.168.50.49`  
For Turtlebot "oogway", do:  
`ssh ubuntu@192.168.50.20`  

It is recommended to use SSH FS VScode extension.  
With SSH FS, you don't need to manually type in the commands every time, and it makes it easy to edit configuration files!  

Sometimes the sensors may not run properly, you need to restart them.  
To do so: Run `turtlebot4-setup`, Navigate to `ROS Setup`, then `Robot Upstart`.  


## OAK-D Lite camera
Once you have SSH into the pi, you may also want to change the configuration file of our OAK-D lite camera.  
It is located at `/opt/ros/humble/share/turtlebot4_bringup/config/oakd_lite.yaml`.  
It will be loaded by oakd.launch.py when starting/restarting "Upstart".  

You can also use `rqt` to view and change camera parameters.  
To change parameters, open "Plugins-Configuration-Dynamic Reconfigure", and drag the left side of the panel open to see list of parameters.  



