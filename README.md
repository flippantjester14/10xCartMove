Got it. Here's the **clean, copy-pasteable, GitHub-ready `README.md`** — fully Markdown compliant **without** boxed formatting, so you can paste it directly into GitHub:

---

# 10xCartMove: Smooth Cartesian Motion for 6DOF Arms

**10xCartMove** is a ROS 2 Humble package for executing **smooth, velocity-controlled Cartesian motion** on 6-DOF robotic arms like the *Franka Emika Panda* or *UR10e*. It implements trapezoidal velocity profiles over 2D paths, supports compliance simulation, and offers real-time joint velocity plotting.

---

## Features

* Smooth Cartesian trajectory over 2D planes
* Trapezoidal velocity profile generation
* Predefined paths: square, spiral, blend
* Adjustable velocity scaling per execution
* Joint velocity publishing via `/joint_states_with_velocity`
* Simulated compliance (constant-force surface interaction)
* RViz + `rqt_plot` support for visualization

---

##  Package Structure

```
ur_ws/
├── src/
│   └── cartesian_trajectory_planner/
│       ├── src/
│       │   ├── cartesian_traj_node.cpp          # Main planner
│       │   └── velocity_estimator_node.cpp      # Velocity publisher
│       ├── launch/
│       │   └── fake_ur10e_planner.launch.py     # Sample demo launch
│       ├── CMakeLists.txt
│       └── package.xml
├── install/
├── build/
├── log/
├── README.md
└── .gitignore
```

---

## How to Run

### 1. Start MoveIt 2 + RViz

```bash
ros2 launch moveit2_tutorials demo.launch.py
```

Make sure `/robot_description` is being published.

---

### 2. Run Cartesian Planner

```bash
ros2 run cartesian_trajectory_planner cartesian_traj_node
```

* Plans a **spiral + square blend**
* Skips execution if path completion < 90%

---

### 3. Run Velocity Estimator

```bash
ros2 run cartesian_trajectory_planner velocity_estimator_node
```

This publishes joint velocity to:

```
/joint_states_with_velocity
```

---

### 4. Visualize with `rqt_plot`

```bash
rqt_plot /joint_states_with_velocity/velocity[0]
```

Replace `[0]` with joint index to plot others.

---

## Compliance Control (Bonus)

* Simulates constant-force contact without torque sensors
* Uses Cartesian velocity override near virtual "surface zone"
* Useful for surface polishing, pushing tasks

---

## Notes

* Path completion consistently ≥ 90%
* Trapezoidal velocity profile via TOTG (MoveIt)
* Path accuracy tested in RViz
* Velocity plotted from timestamp-aligned joint states

---

## Requirements

* ROS 2 Humble
* MoveIt 2
* Install required tools:

```bash
sudo apt install ros-humble-rqt ros-humble-rqt-plot
```

* Tested on **Ubuntu 22.04**

---

## TODO / Future Plans

* [ ] Add parameter or service-based velocity override
* [ ] Export velocity logs to CSV
* [ ] Add path selection from CLI args
* [ ] Replace fake surface zone with real sensor input

---

## Author

Made by [@flippantjester14](https://github.com/flippantjester14)

---
nc
