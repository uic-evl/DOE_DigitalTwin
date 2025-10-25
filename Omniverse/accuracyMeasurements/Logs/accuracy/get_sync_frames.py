# Open v_arm file
v_arm_data = []
with open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Recordings/v_arm_waiting_pass1.txt") as f:
    v_arm_data = f.read().splitlines()

# Open real_arm file
real_arm_data = []
with open("C:/Users/halle/Documents/DigitalTwin/Performance/accuracy/Recordings/real_arm_waiting_pass1.txt") as f:
    real_arm_data = f.read().splitlines()

# Find all executed commands

# Save cmdID, v_arm joints, v_arm endpoint, real_arm joints