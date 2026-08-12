# ROS 2 Bridge — Isaac Sim to Physical MyArm

Connects a simulated firefighter arm in **Isaac Sim** (running on `dt01`) to a
**physical MyArm robot** (MyCobot 300 Pi, connected via a Raspberry Pi) over a
cross-network, cross-ROS-version ROS 2 bridge.

```
Isaac Sim (dt01) ──── ROS 2 topics ──── Raspberry Pi ──── serial ──── MyArm
```

This lets joint motion computed or commanded in simulation drive the real
arm in real time, and streams live telemetry from the physical arm back
into Isaac Sim.

---

## Contents

| File | Purpose |
|------|---------|
| [`PUBLISHER_SETUP.md`](./PUBLISHER_SETUP.md) | How the Isaac Sim publisher works, how to verify it's sending correct data, and how to test the pipeline manually |
| [`CHALLENGES.md`](./CHALLENGES.md) | Full debugging log — network setup, DDS implementation issues, and every failure mode hit while building this bridge, with root causes and fixes |
| [`publisher.py`](./publisher.py) | Runs inside the Isaac Sim Script Editor. Reads live joint state from the USD stage and publishes it to `/isaac_joint_commands` |
| [`link_v3.py`](./link_v3.py) | Runs on the Raspberry Pi. Subscribes to joint commands and drives the physical arm over serial; publishes arm telemetry back to Isaac Sim |

**Start with `PUBLISHER_SETUP.md`** if you're setting this up for the first
time. Refer to `CHALLENGES.md` if something breaks — it's very likely
already been diagnosed there.

---

## Quick Start

**1. Raspberry Pi** — start the arm controller:
```bash
python3 link_v3.py
```
Wait for:
```
Physical arm controller ready. Listening on /isaac_joint_commands | Publishing on /pi_joint_states
```

**2. Isaac Sim (dt01)** — open the Script Editor, load `publisher.py`, and run it.
Confirm topics are visible:
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

**3. Verify data is flowing:**
```bash
ros2 topic echo /pi_joint_states
```

Full setup, environment variables, and network config are documented in
`CHALLENGES.md`.

---

## Architecture Notes

- **ROS 2 versions differ across machines**: the Pi runs **Galactic** (EOL),
  dt01 runs **Kilted**. This cross-version gap is the source of most issues
  documented in `CHALLENGES.md` — particularly DDS serialization
  incompatibilities.
- **Topics:**
  - `/isaac_joint_commands` (dt01 → Pi) — commands from Isaac Sim to the physical arm
  - `/pi_joint_states` (Pi → dt01) — live telemetry from the physical arm back to Isaac Sim
- **Units:** all joint positions are transmitted in **radians**. Isaac Sim's
  USD joint state is read in degrees and converted before publishing.
- **Joint names:** Isaac Sim's USD joint names (e.g. `joint1_to_base`) are
  remapped to the Pi controller's expected names (`joint1`...`joint7`)
  before publishing — see `JOINT_NAME_MAP` in `publisher.py`.

---

## Known Limitations

See `CHALLENGES.md` for full details. Summary:

- Pi remains on ROS 2 Galactic (EOL); long-term fix is upgrading to a
  supported LTS distro (e.g. Humble), not yet done.
- A harmless DDS serialization warning (`string data is not null-terminated`)
  may still appear in Pi logs due to the Galactic/Kilted version gap. It does
  not affect data integrity.
- No zenoh bridge — abandoned due to DDS-layer incompatibility between
  `zenoh-bridge-dds` (Cyclone DDS) and dt01 (Fast DDS). Direct Fast DDS
  unicast is used instead.
