# ROS2 Assignment 2 – Safety Controller with Services

This project implements a simple ROS2 safety system for a mobile robot in simulation.
A user interface node sends velocity commands, while a safety node monitors laser data
and prevents collisions by enforcing a safety threshold.

---

## Nodes

### 1. UI Node (`ui`)
- Publishes user velocity commands on `/cmd_vel_user`
- Allows the user to:
  - Set linear and angular velocity
  - Change the safety threshold via service
  - Request the average of the last 5 velocity commands

### 2. Safety Node (`safety`)
- Subscribes to:
  - `/cmd_vel_user` (user commands)
  - `/scan` (LaserScan)
- Publishes:
  - `/cmd_vel` (safe velocity sent to the robot)
  - `/obstacle_info` (distance and direction of closest obstacle)
- If an obstacle is closer than the threshold:
  - Publishes inverse velocity to move the robot back to a safe zone
- Stores the last 5 user commands and provides their average via service

---

## Custom Interfaces

### Services
- `SetThreshold.srv`  
  Sets the safety distance threshold.

- `GetAverages.srv`  
  Returns the average linear and angular velocity of the last 5 user commands.

### Message
- `ObstacleInfo.msg`  
  Contains:
  - distance to closest obstacle
  - direction (`Left`, `Right`, `Front`)
  - current threshold

---

## How to Run
### 1. Build the workspace
From the root of your ROS2 workspace:

```bash
colcon build
source install/local_setup.bash
```

### 2. Launch simulation and safety node
Start the Gazebo simulation and the safety controller node using the launch file:
```bash
ros2 launch assignment2_rt assignment2.launch.py
```
The launch file starts:
- the robot simulation
- the safety node

### 3. Run the UI node
The UI node requires terminal input and must be run in a separate terminal:
```bash
ros2 run assignment2_rt ui
```
Follow the on-screen instructions to:
- send velocity commands
- change the safety threshold
- request average velocities
