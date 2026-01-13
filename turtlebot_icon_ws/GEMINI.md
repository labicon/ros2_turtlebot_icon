# Turtlebot Icon Workspace

## Project Overview
This is a ROS 2 workspace containing the `turtlebot_icon` package. It is designed for multi-robot systems (specifically Turtlebots) to perform consensus algorithms and synchronized data collection (RGB, Depth, Vicon Pose).

The system integrates **Redis** as a shared memory bridge for inter-process communication and state management between ROS nodes and external applications.

## Key Features
*   **Consensus Algorithm:** `consensus_node` implements a distributed consensus protocol, sharing state (`theta`, `uncertainty`) between agents via ROS topics and storing local/neighbor states in Redis.
*   **Data Synchronization:** `sub_redis_node` subscribes to OAK-D camera feeds (RGB, Depth, Stereo) and Vicon pose data. It synchronizes these streams by timestamp, visualizes them, and saves the aligned data to disk and Redis.
*   **Redis Integration:** Uses a local Redis server (`localhost:6379`) to cache images, poses, and consensus variables.

## Prerequisites
*   **ROS 2:** (e.g., Humble, Foxy)
*   **Redis Server:** Must be running locally on port 6379.
    ```bash
    sudo apt install redis-server
    sudo systemctl start redis-server
    ```
*   **Python Dependencies:**
    *   `redis`
    *   `opencv-python`
    *   `numpy`
    *   `cv_bridge`

## Building and Setup
A helper script is provided to build the workspace and source the setup file.

1.  **Build and Source:**
    ```bash
    ./build_N_source.sh
    ```
    *This script runs `colcon build --symlink-install` and sources `install/setup.bash`.*

## Running Nodes

### 1. Consensus Node
Runs the consensus logic for a specific bot, communicating with a neighbor.

```bash
ros2 run turtlebot_icon consensus_node <bot_name> <neighbor_bot_name>
```
*   **Example:** `ros2 run turtlebot_icon consensus_node miriel2 miriel1`
*   **Topics:** Publishes/Subscribes to `/<bot_name>/consensus`.

### 2. Subscriber & Redis Node (Data Collection)
Subscribes to camera and pose topics, synchronizes data, and saves it.

```bash
ros2 run turtlebot_icon sub_redis_node <bot_name> <save_directory_name>
```
*   **Example:** `ros2 run turtlebot_icon sub_redis_node miriel test_run_01`
*   **Inputs:**
    *   `/vicon/<bot_name>/<bot_name>/pose`
    *   `/<bot_name>/oakd/rgb/image_raw/compressed`
    *   `/<bot_name>/oakd/stereo/image_raw`
*   **Output:** Saves data to `./saved_data/<save_directory_name>/` and updates Redis keys (`rgb_image`, `depth_image`, `vicon_data`).

### 3. Other Nodes
*   `realsense_node`: Likely for RealSense camera integration (usage similar to `sub_redis_node`).
*   `consensus_pub` / `consensus_sub`: Helper or test nodes for specific publish/subscribe parts of the consensus logic.

## Directory Structure
*   `src/turtlebot_icon/turtlebot_icon/`: Python source code for ROS nodes.
*   `saved_data/`: Default output directory for collected data.
*   `build_N_source.sh`: Build helper script.
