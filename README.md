# Catkin Workspace

This project sets up a ROS Noetic workspace using `catkin build`. Follow the instructions below to install ROS Noetic and configure your workspace.

## Prerequisites

- **Operating System:** Ubuntu 20.04 or compatible  
- **Required Dependencies:** ROS Noetic  

Make sure your system is ready for ROS Noetic. Check the official installation guide here:  
[ROS Noetic Installation - Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)

---

### Step 1: Install ROS Noetic

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
After building, source the workspace:


```bash
source devel/setup.bash
```

---

### 📦 **Step 4: Clone and Setup the Project**

```markdown
## Step 4: Clone and Setup the Project

Clone your ROS project (if not already cloned) into the `src` directory:

```bash
cd ~/catkin_workspace/src
git clone <your-repo-url>
cd ~/catkin_workspace
catkin build
source devel/setup.bash
```

---


## Step 5: Run the Demo

To run the ROS–Unity integration, you need to launch **three components**:

1. **RViz Visualization**
2. **Mover Node (Python script)**
3. **ROS-TCP Endpoint**

Run the following commands in separate terminals (or tabs):

### Terminal 1 – Launch RViz
```bash
roslaunch ur10e_rg2_moveit demo.launch
```

### Terminal 2 – Run Mover Script
```bash
rosrun ur10e_rg2_moveit mover.py
```


### Terminal 3 – Start ROS-TCP Endpoint
```bash
roslaunch ROS_TCP_ENDPOINT endpoint.launch
```

---


## Step 6: Test the Setup

In Unity, open your project with the **ROS–TCP Connector**.  
Ensure the IP address and port in Unity match the ROS `endpoint.launch` configuration.  
Once connected, you should be able to see live movement and visualization in RViz.


