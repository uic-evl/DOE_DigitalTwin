# MyArm Physical Robot — ROS 2 Network Bridge

## Project Overview

Connect a physical MyArm robot on a Raspberry Pi to Isaac Sim running on dt01
over a cross-network, cross-ROS-version ROS 2 bridge.

```
Isaac Sim (dt01) ──── ROS 2 topics ──── Pi ──── serial ──── MyArm
```

---

## Hardware

| Device | Role |
|--------|------|
| Raspberry Pi (er@er) | Runs arm controller, connected to MyArm via serial |
| dt01 (username@dt01) | Runs Isaac Sim, sends commands, receives telemetry |
| MyArm robot | MyCobot 300 Pi|

---

## Network

| Machine | IP |
|---------|----|
| Pi | 10.31.81.10 |
| dt01 | 140.221.17.49 (check ifconfig for private network) |

- Pi can ping dt01 
- dt01 can ping Pi 
- Serial port: `/dev/ttyAMA0` at `115200` baud

---

## Software

| Machine | OS | ROS Version | Default RMW |
|---------|----|-------------|-------------|
| Pi | Raspberry Pi OS | ROS 2 Galactic | Fast DDS |
| dt01 | Ubuntu | ROS 2 Kilted | Fast DDS |

---

## ROS 2 Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/isaac_joint_commands` | dt01 → Pi | `sensor_msgs/msg/JointState` | Joint commands from Isaac Sim to physical arm |
| `/pi_joint_states` | Pi → dt01 | `sensor_msgs/msg/JointState` | Real arm telemetry back to Isaac Sim |

### Message format
- `position`: 7 joint angles in **radians**
- `name`: **must be empty list `[]`** — see Known Issues
- `velocity`: empty list
- `effort`: empty list

---

## What Works

### Serial connection
- MyArm connects on `/dev/ttyAMA0` at `115200` baud
- `mc.get_angles()` returns 7 joint angles in degrees
- `mc.send_angles()` moves the arm

### ROS 2 topics visible across network
- `/pi_joint_states` visible on dt01
- `/isaac_joint_commands` visible on Pi
- Telemetry streams at 10Hz from Pi to dt01

### Fast DDS unicast config
- Both machines use Fast DDS with explicit unicast peer config
- No multicast needed
- No zenoh bridge needed
- No extra processes needed

### Arm telemetry streaming
- `ros2 topic echo /pi_joint_states` on dt01 shows live joint angles

---

## What Did Not Work and Why

### Pi can ping dt01 but dt01 can't ping Pi
**Problem:** `dt01 run under private network`
**MAIN FIX:** `Connected ethernet from Linux machine running Isaac Sim to the Switch that connected to Raspberry Pi`

###  Zenoh bridge (zenoh-bridge-dds)
**Problem:** `Error creating DDS Topic: Timeout` on every topic
**Cause:** `zenoh-bridge-dds` binary compiled against Cyclone DDS internally,
but dt01 uses Fast DDS. They cannot share the same DDS layer.
**Attempted fix:** Switch dt01 to `rmw_zenoh_cpp` with `rmw_zenohd` router.
**Result:** Bridge connected but `rmw_zenoh_cpp` and `zenoh-bridge-dds` use
different Zenoh key expression formats — topics not visible across the bridge.
**Final decision:** Abandoned zenoh entirely. Used Fast DDS unicast directly.

###  rmw_zenoh_cpp on dt01
**Problem:** Topics from Pi not visible on dt01 even with zenohd router running
**Cause:** `zenoh-bridge-dds` uses DDS key format, `rmw_zenoh_cpp` uses a
different Zenoh key format. They do not interoperate.
**Fix:** Abandoned. Used Fast DDS unicast instead.

###  Cyclone DDS on dt01
**Problem:** `librmw_cyclonedds_cpp.so: cannot open shared object file`
**Cause:** Cyclone DDS not installed on dt01, no sudo access to install it.
**Fix:** Kept Fast DDS on dt01.

###  Cyclone DDS XML config (old schema)
**Problem:** `unknown element` XML parse error
**Cause:** ROS Galactic ships an older Cyclone DDS version with different XML tags.

**Wrong config:**
```xml
<NetworkInterface name="eth0" priority="default" multicast="false"/>
```

**Correct config for Galactic:**
```xml
<NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
<AllowMulticast>false</AllowMulticast>
```

###  Fast DDS bad_alloc crash on Pi
**Problem:** `terminate called after throwing an instance of std::bad_alloc`
**Cause:** Pi had zero swap configured. Fast DDS allocates large memory pools. When receiving a message with a name string array, memory allocation failed.
**Fix 1:** Enable 1GB swap on Pi (see setup below).
**Fix 2:** Send commands with `name: []` empty list to avoid string allocation.
**Fix 3:** Use Cyclone DDS on Pi — uses far less memory than Fast DDS.

###  Cyclone DDS serialization warning on Pi
**Problem:**
```
'string data is not null-terminated, at serdata.cpp:354'
```
**Cause:** Known bug in ROS Galactic `rmw_cyclonedds_cpp` 0.22.6 when deserializing string arrays sent from a newer ROS version (Kilted).
**Fix:** Send `name: []` empty list from dt01. The warning still appears but is harmless — data flows correctly.

Suppress the warning:
```bash
python3 link_v3.py 2>&1 | grep -v "rcutils\|serdata\|error_handling\|rcutils_reset\|overwritten"
```

###  dt01 could not ping Pi initially
**Problem:** dt01 could not reach Pi, only Pi could reach dt01.
**Cause:** Network routing issue — Pi was behind NAT or on a different subnet.
**Fix:** Network was reconfigured. Both machines can now reach each other. This was the root cause of many bridge failures.

###  Fast DDS XML invalid tag
**Problem:** `Invalid element found into 'discoverySettingsType': use_SIMPLE_EndpointDiscoveryProtocol`
**Cause:** Tag does not exist in the Fast DDS version on Galactic Pi.
**Fix:** Remove the tag. Minimal working XML shown below.

###  FASTRTPS_DEFAULT_PROFILES_FILE deprecation warning
**Problem:**
```
FASTRTPS_DEFAULT_PROFILES_FILE value is used for participant configuration,
but it is deprecated
```
**Cause:** ROS Kilted uses the new variable name `FASTDDS_DEFAULT_PROFILES_FILE`.
**Fix:** Use `FASTDDS_DEFAULT_PROFILES_FILE` on dt01. Both variables work, the warning is harmless.

---

## Working Configuration

### Pi — Fast DDS unicast config

File: `/home/er/fastdds_unicast.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="unicast_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>10.31.81.10</address>
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>140.221.17.49 (changed to private network)</address>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
      <defaultUnicastLocatorList>
        <locator>
          <udpv4>
            <address>10.31.81.10</address>
          </udpv4>
        </locator>
      </defaultUnicastLocatorList>
    </rtps>
  </participant>
</profiles>
```

### dt01 — Fast DDS unicast config

File: `/home/brianta/fastdds_unicast.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="unicast_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>140.221.17.49 (changed to private network)</address>
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>10.31.81.10</address>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
      <defaultUnicastLocatorList>
        <locator>
          <udpv4>
            <address>140.221.17.49 (changed to private network)</address>
          </udpv4>
        </locator>
      </defaultUnicastLocatorList>
    </rtps>
  </participant>
</profiles>
```

### For CycloneDDS XML 
```
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" priority="default" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>10</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="10.31.81.10"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>



```

---

## Environment Variables

### Pi `~/.bashrc`

```bash
source /opt/ros/galactic/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/home/er/fastdds_unicast.xml

or

source /opt/ros/galactic/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/er/cyclone_dds.xml (you'll need to create another xml file)
python3 ~/Desktop/brian/link_v3.py
```

### dt01 `~/.bashrc`

```bash
source /opt/ros/kilted/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/home/brianta/fastdds_unicast.xml
```




---

## Daily Startup Procedure

### Step 1 — Pi

```bash
python3 ~/Desktop/brian/link_v3.py 2>&1 | grep -v "rcutils\|serdata\|error_handling\|rcutils_reset\|overwritten"
```

Wait for:
```
Physical arm controller ready. Listening on /isaac_joint_commands | Publishing on /pi_joint_states
```

### Step 2 — dt01 verify topics

```bash
ros2 topic list
```

Expected:
```
/isaac_joint_commands
/parameter_events
/pi_joint_states
/rosout
```

### Step 3 — dt01 verify telemetry

```bash
ros2 topic echo /pi_joint_states \
  --qos-reliability reliable \
  --qos-durability volatile
```

### Step 4 — dt01 send test command

```bash
ros2 topic pub --once /isaac_joint_commands sensor_msgs/msg/JointState \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, \
    name: [], \
    position: [0.0, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0], \
    velocity: [], \
    effort: []}" \
  --qos-reliability reliable \
  --qos-durability volatile
```

Pi should print:
```
[INFO] Commanding arm (deg): [0.0, 11.46, 5.73, 0.0, 0.0, 0.0, 0.0]
```

---

## Isaac Sim Integration Requirements

When connecting Isaac Sim to this pipeline:

1. Publish to `/isaac_joint_commands` with:
   - `name: []` — empty list, do not send joint names
   - `position`: 7 values in radians
   - `velocity: []`
   - `effort: []`
   - QoS: `reliability=RELIABLE`, `durability=VOLATILE`

2. Subscribe to `/pi_joint_states` with:
   - QoS: `reliability=RELIABLE`, `durability=VOLATILE`
   - `position`: 7 values in radians

---

## Key Files

| File | Location | Purpose |
|------|----------|---------|
| `link_v3.py` | Pi `~/Desktop/brian/` | Main arm controller node |
| `fastdds_unicast.xml` | Pi `~/` | Fast DDS unicast peer config |
| `fastdds_unicast.xml` | dt01 `~/` | Fast DDS unicast peer config |

---

## Known Issues

| Issue | Status | Workaround |
|-------|--------|------------|
| Cyclone DDS string serialization warning on Pi | Non-fatal | Filter with grep or send `name: []` |
| Fast DDS bad_alloc on Pi | Fixed | Check publisher script |
| FASTRTPS_DEFAULT_PROFILES_FILE deprecation warning | Non-fatal | Use `FASTDDS_DEFAULT_PROFILES_FILE` |
| dt01 has no Cyclone DDS and no sudo | Permanent | Use Fast DDS on dt01 |
| zenoh-bridge-dds incompatible with Fast DDS | Abandoned | Use direct Fast DDS unicast |
