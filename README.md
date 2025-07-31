# Dual arm hotcell simulation (isaac lab)
This project sets up a dual-arm robot simulation using NVIDIA isaac lab. it includes a custom environment (`hotcell_env.py`) and an inverse kinematics controller (`rule_based_controller.py`) to move two arms in parallel across multiple environments.
## Files
- **hotcell_env.py**
  defines a gym-compatible isaac lab environment with two robotic arms and target cubes. each arm interacts with its own cube, and the scene supports randomized spawning, reset logic, and multi-env replication.
- **arm_controller.py**
  uses pytorch kinematics and a multi-stage control strategy to solve inverse kinematics and return joint targets for each arm. the controller guides each arm through hovering, grasping, lifting, and placing stages.

  #### Creators:
  Athena Angara: aanga3@uic.edi
  Yassir Atlas: yatla2@illinois.edu
