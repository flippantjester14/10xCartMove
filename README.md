# 10xCartMove: Smooth Cartesian Motion for 6DOF Arms

`10xCartMove` is a ROS 2 Humble package for executing **smooth, velocity-controlled Cartesian motion** on 6-DOF robotic arms like the Franka Emika Panda or UR10e. It implements trapezoidal profile motion across custom paths with optional compliance simulation and joint velocity visualization.

---

## ✅ Features

- 🔄 Smooth Cartesian trajectory across 2D plane
- 📐 Trapezoidal velocity profile generation
- 🎯 Custom paths (square, spiral, blended)
- ⚙️ Velocity scaling per execution
- 🔌 Joint velocity published via `/joint_states_with_velocity`
- 🧱 Simulated compliance control (force via velocity override)
- 🖥️ RViz visual feedback + rqt_plot support

---

## 📦 Package Structure

ur_ws/
├── src/
│ └── cartesian_trajectory_planner/
│ ├── src/
│ │ ├── cartesian_traj_node.cpp # Trajectory planner
│ │ └── velocity_estimator_node.cpp # Publishes joint velocities
│ ├── launch/
│ │ └── fake_ur10e_planner.launch.py # For Panda testing
│ ├── CMakeLists.txt
│ └── package.xml
├── install/
├── build/
├── log/
├── README.md
└── .gitignore

▶️ Run the Planner
1. Start MoveIt 2 demo (e.g., for Panda)

ros2 launch moveit2_tutorials demo.launch.py

Ensure /robot_description is published and RViz is open.
2. Run Cartesian Trajectory Node

ros2 run cartesian_trajectory_planner cartesian_traj_node

    Plans a spiral + square blend

    Prints path completion percentage (must exceed 90%)

    Executes only on success

3. Launch Velocity Estimator

ros2 run cartesian_trajectory_planner velocity_estimator_node

This publishes real-time joint velocity as:

/joint_states_with_velocity

4. Plot Joint Velocities

rqt_plot /joint_states_with_velocity/velocity[0]

You can change index ([1], [2], etc.) to see different joints.
💡 Compliance Control

    Simulated compliance using Cartesian velocity override when end-effector reaches target “surface zone”

    This mimics constant-force pushing without requiring direct torque control

    Useful for surface interaction testing or polishing behaviors

📈 Accuracy Notes

    Path planning validated for 90%+ completion

    Trapezoidal profile interpolated over EEF path

    /joint_states_with_velocity is timestamp-aligned for accurate graphing

    Path can be edited easily (e.g., spiral radius, offset)

📌 Future Work

Add parameter-based velocity scaling

GUI slider or service for real-time override

Export joint velocity logs for offline plotting

    Replace fake surface check with real proximity sensor

🧠 Author

Made with ⚙️ and ❤️ by @flippantjester14
for the 10x Motion Planning Challenge
🧪 Requirements

    ROS 2 Humble

    MoveIt 2

    rqt_plot: sudo apt install ros-humble-rqt-plot

    Tested on Ubuntu 22.04 (x86_64)
