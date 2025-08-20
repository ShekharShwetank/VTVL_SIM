# VTVL Rocket Simulator - Progress Log

Last Updated: August 20, 2025

1. Current Project Status

Phase 1 (1-DoF Vertical Hopper): Complete. A C++ physics simulator and a PID controller for altitude hold are functional.

Phase 2 (3-DoF Planar Rocket): In Progress. The physics simulator and controller have been upgraded to a 3-DoF planar model (x, z, pitch). Attitude stabilization is implemented but requires final debugging.

Visualization: A Gazebo-based visualization is set up. A kinematic rocket model is successfully spawned into a custom world and is driven by the state output of the C++ physics simulator.

Environment: The entire development environment is containerized using a VS Code Dev Container for 100% reproducibility.

```
VTVL_SIM/
├── .devcontainer/
│   ├── devcontainer.json
│   └── Dockerfile
├── .gitignore
└── rocket-sim/
    ├── build/
    ├── install/
    ├── log/
    ├── models/
    │   └── simple_rocket/
    │       ├── model.config
    │       └── model.sdf
    ├── src/
    │   ├── sim/
    │   │   ├── CMakeLists.txt
    │   │   ├── package.xml
    │   │   ├── launch/
    │   │   │   └── sim_3dof.launch.py
    │   │   ├── scripts/
    │   │   │   └── visualizer_node.py
    │   │   └── src/
    │   │       ├── planar_controller.cpp
    │   │       └── sim_dynamics.cpp
    │   └── vtvl_msgs/
    │       ├── CMakeLists.txt
    │       ├── package.xml
    │       └── msg/
    │           ├── ActuatorControls.msg
    │           └── VehicleState.msg
    └── worlds/
        └── basic_pad.world
```
