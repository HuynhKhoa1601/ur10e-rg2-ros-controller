# Catkin Workspace

This project sets up a ROS Noetic workspace using `catkin build`. Follow the instructions below to install ROS Noetic and configure your workspace.

## Prerequisites

- **Operating System:** Ubuntu 20.04 or compatible  
- **Required Dependencies:** ROS Noetic  

Make sure your system is ready for ROS Noetic. Check the official installation guide here:  
[ROS Noetic Installation - Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)

---

## ## Step 2: Install ROS Noetic

Follow the [official ROS installation guide](https://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS Noetic. 

Ensure you run the following command after installation to source ROS properly:

```bash
cd ~/catkin_workspace
source /opt/ros/noetic/setup.bash
```

### Step 2: Install catkin tools and build

```bash
pip3 install --user git+https://github.com/catkin/catkin_tools.git
catkin build
```
