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
- **Gazebo model_states** used as ground truth  

---

## 🎯 Objectives

- Create **custom simulation environments** in Gazebo  
- Program **short and complex trajectories** for TurtleBot3  
- Automate **trajectory execution and data recording**  
- Record sensor data from `/odom`, `/imu`, `/scan`  
- Compare estimated trajectories with ground truth  
- Ensure **synchronized and reproducible runs**  

---

## 📂 Project Structure

turtlebot3-trajectory-estimation/
├── worlds/ # Custom .world files for Gazebo
├── launch/ # Launch files for trajectory + data recording
├── scripts/ # Python scripts for robot motion
├── bags/ # rosbag output files (not included)
├── results/ # Analysis plots and CSVs
└── README.md # Project documentation

---

## 🚀 Usage

### 1. Add your custom world
Place your `.world` file in `turtlebot3_gazebo/worlds/`  
Reference it in a launch file in `turtlebot3_gazebo/launch/`.

Launch the world:
```bash
roslaunch turtlebot3_gazebo custom_world.launch
```
### 2. Execute a trajectory and record data
Run a custom launch file from your own package:
```bash
roslaunch my_turtlebot execute_trajectory.launch
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
Implement sensor fusion to improve trajectory accuracy
Visualize and compare trajectories from each sensor
Compute estimation errors (e.g. RMSE vs. ground truth)
