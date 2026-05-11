# UR10e Industrial Digital Twin — ROS 2

A fully simulated industrial robot arm (Universal Robots UR10e) running as a digital twin in ROS 2 Jazzy, with autonomous pick-and-place capability powered by MoveIt 2 motion planning.

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![MoveIt](https://img.shields.io/badge/MoveIt_2-Latest-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## Demo

https://github.com/user-attachments/assets/demo-placeholder

> The UR10e arm autonomously executes a full pick-and-place cycle — pre-pick → pick → grasp → lift → transport → place → release → retreat — in both Gazebo simulation and RViz2 digital twin view simultaneously.

## What This Project Does

This project simulates a real-world industrial pick-and-place operation using the same ROS 2 Control + MoveIt 2 stack that runs on physical UR10e deployments. The arm plans collision-free trajectories and executes them in a physics-simulated Gazebo environment, while RViz2 provides a live digital twin view of the robot state.

**Key features:**
- Autonomous 9-step pick-and-place cycle with zero human input
- MoveIt 2 motion planning with OMPL (collision-free trajectories)
- Gazebo Harmonic physics simulation with ROS 2 Control hardware interface
- Live RViz2 digital twin visualization mirroring the simulation
- Configurable poses — easily adapt to any workspace layout
- Single-command launch for each component

## System Architecture

```
┌────────────────────┐     ┌──────────────────────┐     ┌────────────────────┐
│   Gazebo Harmonic  │◄───►│   ROS 2 Control      │◄───►│     MoveIt 2       │
│   (Physics Sim)    │     │   (Joint Interface)  │     │  (Motion Planner)  │
└────────────────────┘     └──────────────────────┘     └────────┬───────────┘
                                                                 │
                           ┌──────────────────────┐     ┌────────┴───────────┐
                           │      RViz2           │     │  pick_and_place.py │
                           │   (Digital Twin)     │     │  (Python Node)     │
                           └──────────────────────┘     └────────────────────┘
```

## Prerequisites

- **OS:** Ubuntu 24.04
- **ROS 2:** Jazzy Jalisco
- **RAM:** 16 GB recommended
- **GPU:** Any (Gazebo uses software rendering if no dedicated GPU)

## Installation

### 1. Install ROS 2 Jazzy (if not already installed)

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### 2. Install required packages

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-moveit -y
```

### 3. Clone and build

```bash
# Create workspace
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src

# Clone UR driver and Gazebo simulation packages
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git

# Clone this project
git clone https://github.com/Dinoopdas/ur10e-industrial-digital-twin-ros2.git

# Install dependencies
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage

Open three terminals and run in order:

### Terminal 1 — Gazebo Simulation
```bash
source ~/robot_ws/install/setup.bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur10e
```
Wait for the Gazebo window to fully load with the UR10e visible.

### Terminal 2 — MoveIt 2 + RViz2 (Digital Twin)
```bash
source ~/robot_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true use_sim_time:=true
```
Wait for RViz2 to open and display the arm with the interactive marker.

### Terminal 3 — Autonomous Pick-and-Place
```bash
source ~/robot_ws/install/setup.bash
ros2 run ur10e_industrial_twin pick_and_place --ros-args -p use_sim_time:=true
```

The arm will execute the full pick-and-place cycle autonomously.

## Pick-and-Place Sequence

| Step | Action | Description |
|------|--------|-------------|
| 1 | Gripper open | Prepare for pick |
| 2 | Pre-pick | Move above the object |
| 3 | Pick | Descend to object height |
| 4 | Gripper close | Grasp the object |
| 5 | Lift | Rise back to safe height |
| 6 | Pre-place | Move above the destination |
| 7 | Place | Descend to destination height |
| 8 | Gripper open | Release the object |
| 9 | Retreat | Rise to safe height |

## Customizing Poses

Edit the pose definitions in `ur10e_industrial_twin/pick_and_place.py`:

```python
PRE_PICK_POSE = Pose(
    position=Point(x=0.4, y=0.2, z=0.35),
    orientation=Quaternion(x=0.0, y=0.7071, z=0.0, w=0.7071),
)
```

All positions are in the `base_link` frame (meters). The orientation `Quaternion(0, 0.7071, 0, 0.7071)` points the tool straight down — standard for pick-and-place.

## Project Structure

```
ur10e_industrial_twin/
├── ur10e_industrial_twin/
│   ├── __init__.py
│   └── pick_and_place.py      # Autonomous pick-and-place node
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Technical Details

- **Motion planning:** OMPL via MoveIt 2 with 10 planning attempts, 10s timeout
- **Velocity:** 30% max scaling for smooth industrial-speed motion
- **Concurrency:** MultiThreadedExecutor + ReentrantCallbackGroup pattern to avoid ROS 2 executor deadlocks
- **Sim time:** Synchronized with Gazebo simulation clock via `use_sim_time` parameter

## Technologies

- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- MoveIt 2
- ROS 2 Control
- Universal Robots ROS2 Driver
- Python 3.12
- OMPL (Open Motion Planning Library)

## What's Next

This project is part of a larger industrial digital twin learning path:
- **Project 2:** Process & sensor monitoring simulation (virtual flow, pressure, temperature sensors)
- **Project 3:** AMR navigator for a simulated industrial facility (Nav2 + SLAM)
- **Project 4:** Real-time digital twin dashboard (rosbridge + web UI)
- **Project 5:** Full loading/unloading facility digital twin (capstone)

## Author

**Dinoop** — Engineering professional transitioning into industrial robotics and digital twin development. Background in engineering procurement and document control for large-scale industrial projects (oil & gas, rail infrastructure).

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
