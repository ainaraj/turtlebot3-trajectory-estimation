# 🧭 TurtleBot3 Trajectory Estimation in Simulation

Trajectory estimation of a TurtleBot3 in Gazebo using ROS1 and onboard sensor data (LiDAR, odometry, IMU).  
This project was developed as part of a robotics course to explore mobile robot localization using sensor data in a simulated environment.

---

## 🛠️ Tools & Technologies

To ensure compatibility and stability, the project was developed using **VirtualBox** with Ubuntu 20.04.

- **ROS1 (Noetic)** on Ubuntu 20.04  
- **Gazebo** for 3D simulation  
- **Python** for trajectory control  
- **rosbag** for data logging  
- `/gazebo/model_states` used as ground truth  
 
---

## 🎯 Objectives

- Create **custom simulation environments** in Gazebo  
- Program **short and complex trajectories** for TurtleBot3  
- Automate **trajectory execution and data recording**  
- Record sensor data from `/odom`, `/imu`, and `/scan`  
- Compare estimated trajectories with ground truth  
- Ensure **synchronized and reproducible runs**  

---

## 📂 Project Structure

turtlebot3-trajectory-estimation/
- worlds/ # Custom .world files for Gazebo
- launch/ # Launch files for trajectory + data recording
- scripts/ # Python scripts for robot motion
- bags/ # rosbag output files (not included)
- results/ # Some analysis plots 
- README.md # Project documentation

---

## 🚀 Usage

### 0. Start ROS Core

Before anything else, make sure to start `roscore` in a separate terminal:

```bash
roscore
```

### 1. Add your custom world
Place your `.world` file in `turtlebot3_gazebo/worlds/`  
Reference it in a launch file in `turtlebot3_gazebo/launch/`.
Then launch the simulation:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_plaza_world.launch
```

### 2. Custom Package Setup
Create a catkin workspace if you don’t already have one:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

Then, create your custom package (e.g., my_turtlebot) inside src/ with the following structure:
my_turtlebot/
├── bags/             # For storing rosbag recordings
├── launch/           # Launch files for executing trajectories
├── scripts/          # Python scripts defining robot motion
├── CMakeLists.txt
└── package.xml

Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


### 3. Execute a trajectory and record data
Run a custom launch file from your own package:
```bash
roslaunch execute_trajectory.launch
```
This launch file will:
- Start a Python script for a predefined trajectory
- Begin recording /odom, /imu, and /scan with rosbag
- Stop recording automatically at the end of the motion
- Ensure that robot motion and data recording are synchronized, resulting in accurate and reliable timestamps

📊 Data Extraction (Offline)
You can extract CSV files from .bag files for analysis:
```bash
rostopic echo -b my_file.bag -p /odom > odom.csv
```

🔄 Next Steps ?
- Implement sensor fusion to improve trajectory accuracy
- Visualize and compare trajectories from each sensor
- Compute estimation errors (e.g. RMSE vs. ground truth)
