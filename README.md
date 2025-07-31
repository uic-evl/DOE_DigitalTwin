# Dual Arm Hotcell Simulation (Isaac Lab)
This project sets up a dual-arm robot simulation using NVIDIA isaac lab. it includes a custom environment (`hotcell_env.py`) and an inverse kinematics controller (`rule_based_controller.py`) to move two arms in parallel across multiple environments.
## Files
- **hotcell_env.py**
  Defines a gym-compatible isaac lab environment with two robotic arms and target cubes. Each arm interacts with its own cube, and the scene supports randomized spawning, reset logic, and multi-env replication.
- **rule_based_controller.py**
  Uses pytorch kinematics and a multi-stage control strategy to solve inverse kinematics and return joint targets for each arm. The controller guides each arm through hovering, grasping, lifting, and placing stages.

#### Creators:
- Athena Angara: aanga3@uic.edu
- Yassir Atlas: yatla2@illinois.edu
