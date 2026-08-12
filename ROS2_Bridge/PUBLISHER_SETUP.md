# Publishing Isaac Sim Joint Angles to Raspberry Pi via ROS 2

This guide explains how to publish joint angles from Isaac Sim to a Raspberry Pi arm controller using `sensor_msgs/msg/JointState`.

## Problem Summary

The Raspberry Pi originally showed DDS errors like:

```
invalid data size, string data is not null-terminated
```

Later, after fixing topic/type issues, the Pi reported:

```
failed to send angles to arm: 'h' format requires -32768 <= number <= 32767
```

This indicates two separate issues:

1. ROS 2 topic/type mismatch
2. Joint angle values out of range for the Pi controller

## Root Causes

### 1. Topic mismatch

Isaac Sim was publishing to:

```
/isaac_joint_states
```

But the Pi controller was subscribing to:

```
/isaac_joint_commands
```

So the Pi never received the intended Isaac messages.

### 2. Wrong joint names

Isaac Sim produced joint names like:

```
joint1_to_base
joint2_to_joint1
joint3_to_joint2
...
```

But the Pi controller expects:

```
joint1
joint2
joint3
joint4
joint5
joint6
joint7
```

### 3. Wrong units

Isaac Sim joint readings appeared to be in **degrees**, while the Pi controller expects **radians**.

Example:

- Isaac value: `120.57°`
- In radians: `2.10`

If the Pi internally does something like:

```python
struct.pack('h', int(angle * 1000))
```

Then:

- `120.57 * 1000 = 120570` → out of range
- `2.10 * 1000 = 2100` → valid

## Working Approach

Use a ROS 2 publisher in Isaac Sim that:

- Publishes to `/isaac_joint_commands`
- Sends `sensor_msgs/msg/JointState`
- Includes only the 7 arm joints
- Remaps joint names to `joint1` ... `joint7`
- Converts degrees to radians before publishing

## Full Isaac Sim Publisher Code

```
see ./publisher.py
```

## How to Verify

### 1. Check topic list

On both machines:

```bash
ros2 topic list 
```

Expected relevant topics:

```
/isaac_joint_commands
/isaac_joint_states
/pi_joint_states
```

### 2. Check topic connections

```bash
ros2 topic info /isaac_joint_commands -v
```

Expected:

- 1 publisher from Isaac side
- 1 subscriber on Pi side
- type: `sensor_msgs/msg/JointState`

### 3. Echo the topic

```bash
ros2 topic echo /isaac_joint_commands
```

You should see messages like:

```
name:
- joint1
- joint2
- joint3
- joint4
- joint5
- joint6
- joint7
position:
- -0.95
- 0.62
- -0.13
- -0.63
- 0.08
- -1.88
- 2.10
```

These values should be in **radians**, not degrees.

## Manual Test Command

If you want to manually test the Pi subscriber:

```bash
ros2 topic pub --rate 1 /isaac_joint_commands sensor_msgs/msg/JointState \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'], \
    position: [0.0, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0], \
    velocity: [], effort: []}" \
  --qos-reliability reliable \
  --qos-durability volatile
```

If this works, then ROS communication is fine and any remaining issue is in the Isaac publisher values or naming.

## Common Failure Modes

### DDS serialization error

Example:

```
string data is not null-terminated, invalid data size
```

**Cause:**

- Publisher and subscriber are not using the same ROS message type
- Stale node on same topic
- Malformed custom message

**Fix:**

- Ensure both sides use `sensor_msgs/msg/JointState`
- Verify with `ros2 topic info -v`

### Failed to send angles to arm

Example:

```
failed to send angles to arm: 'h' format requires -32768 <= number <= 32767
```

**Cause:**

- Values too large after scaling in Pi controller
- Usually due to sending degrees instead of radians

**Fix:**

- Convert Isaac joint values to radians before publishing

## Notes

- Only publish the 7 arm joints unless the Pi explicitly supports gripper joints.
- Use absolute topic names like `/isaac_joint_commands`.
- Keep joint order fixed.
- Ensure joint names exactly match what the Pi expects.
