# 10xCartMove: Smooth Cartesian Motion for 6DOF Arms

This ROS 2 package enables **precise and smooth Cartesian trajectory execution** for 6-DOF robotic arms (e.g., Panda, UR10e) using MoveIt 2. It supports:

- Constant-velocity motion across 2D waypoints
- Trapezoidal profile planning
- Velocity scaling control
- Optional compliance control (simulated force interaction with surface)
- Joint velocity visualization via `/joint_states_with_velocity`

---

## 📦 Package Structure

```bash
ur_ws/
├── src/
│   └── cartesian_trajectory_planner/
│       ├── src/cartesian_traj_node.cpp         # Main Cartesian planner
│       ├── src/velocity_estimator_node.cpp     # Velocity estimator node
│       ├── CMakeLists.txt
│       └── package.xml
├── install/
├── build/
├── log/
├── README.md
└── .gitignore
