# P25: Room Delivery by QR Codes

A TurtleBot3 (waffle_pi) in a Gazebo world with four rooms. Each room has a QR sign on its door wall. The robot drives to a room with Nav2, reads the sign through its camera to confirm it is in the right place, and carries out a list of deliveries.

**What is new in this version**

- A study of how sign size, distance and viewing angle affect reading (D4).
- A wrong-room check: if two signs are swapped, the robot reports a mismatch (D7).
- A fallback for an unreadable sign: rotate to other angles before giving up (D8).
- A mission queue: `trip.py` takes a list of rooms and runs them in order (D6).

Stack: ROS 2, Gazebo (gz sim), Nav2 (Simple Commander), OpenCV (`QRCodeDetector`), `cv_bridge`, Python.

---

## Repository layout

```
qr_delivery/
├── make_world.py          # generates worlds/rooms.sdf and signs/*.png
├── worlds/rooms.sdf       # four rooms, door gaps, QR signs on the walls
├── signs/                 # QR textures (room_101.png ...)
├── launch/rooms.launch.py # Gazebo server + GUI + robot_state_publisher + TurtleBot3 spawn
├── qr_node.py             # camera -> QR decode -> publishes /room_sign, logs sign_obs.csv
├── sign_pose.py           # distance / angle / bearing estimate from the QR corners
├── trip.py                # mission: Nav2 goals, QR confirm, sweep fallback, trips.csv log
├── check_frames.py        # offline detector check on saved frames
├── validate_pose.py       # compares the pose estimate against D4 frames (true values in file names)
├── range_frames/          # D4 test frames, named s<size_cm>_d<dist_m>_a<angle>.png
├── trips.csv              # mission log (one row per trip)
└── sign_obs.csv           # per-frame sign observations from qr_node
```

Rename the files above to match your actual repo if they differ.

---

## Setup

Requirements: Ubuntu, ROS 2 with Gazebo (gz sim) and `ros_gz_sim`, Nav2, `turtlebot3_gazebo` (waffle_pi, which has a camera), `cv_bridge`. No GPU needed.

```bash
pip install opencv-python qrcode[pil] numpy
```

Generate the world and the signs (the `TEX` dict in `make_world.py` swaps sign textures for the wrong-room test):

```bash
python3 make_world.py
```

## Run

Use one terminal per step.

1. **Simulation**
   ```bash
   ros2 launch ~/qr_delivery/launch/rooms.launch.py
   ```
   The robot spawns at (0.7, 0.0). `START` in `trip.py` must match this pose.

2. **Nav2 with a saved map of the rooms** (build the map with the Nav2 "navigating while mapping" tutorial, then save it)
   ```bash
   ros2 launch nav2_bringup bringup_launch.py map:=<path/to/map.yaml> use_sim_time:=true
   ```

3. **QR node**
   ```bash
   python3 qr_node.py
   ```
   Publishes the decoded text on `/room_sign` for every frame that decodes. Parameters: `image_topic`, `sign_m` (sign edge, m), `focal_px`.

4. **Mission**
   ```bash
   python3 trip.py room_103 room_101 room_104
   python3 trip.py room_103 yaw=40        # start with a heading offset (degrees) for the fallback test
   ```
   Results are appended to `trips.csv`. The first run after a header change needs the old file moved aside.

### How `trip.py` works

1. Publishes the start pose, waits for Nav2, and tightens the goal tolerance.
2. For each room, sends a Nav2 goal about 0.9 m in front of the sign, facing it.
3. On arrival, waits 1.5 s for `/room_sign`. If nothing is read, it rotates to ±20°, ±40° and ±60° from the arrival heading (`ANGLES`), waiting for a read after each step.
4. Outcome per trip:
   - `OK`: the read text equals the expected room.
   - `MISMATCH`: a different room was read (the wrong-room check).
   - `UNREADABLE`: nothing was read after all sweeps; the last camera frame is saved as `noread_<room>_<time>.png`.
   - `NAV_FAILED`: Nav2 did not reach the goal.

---

## Results

### D4: reading range (successes out of 5)

**15 cm sign** (rows: distance in m, columns: viewing angle in degrees)

| dist | 0 | 10 | 20 | 30 | 40 | 50 |
|---|---|---|---|---|---|---|
| 0.4 | 1 | 1 | 0 | 0 | 0 | 0 |
| 0.5 | 5 | 5 | 5 | 5 | 4 | 0 |
| 0.6 | 5 | 5 | 5 | 5 | 0 | 0 |
| 0.7 | 5 | 5 | 5 | 0 | 0 | 0 |
| 0.8 | 5 | 3 | 0 | 0 | 0 | 0 |
| 0.9 | 0 | 0 | 1 | 1 | 0 | 0 |

**30 cm sign**

| dist | 0 | 10 | 20 | 30 | 40 | 50 |
|---|---|---|---|---|---|---|
| 0.8 | 5 | 5 | 5 | 5 | 5 | 4 |
| 1.0 | 5 | 5 | 5 | 5 | 5 | 1 |
| 1.2 | 5 | 5 | 5 | 5 | 3 | 0 |
| 1.4 | 5 | 5 | 5 | 0 | 0 | 1 |
| 1.6 | 1 | 0 | 0 | 1 | 0 | 0 |
| 1.8 | 0 | 0 | 2 | 1 | 0 | 0 |

- Reliable range at 0° is 0.5 to 0.8 m for the 15 cm sign and 0.8 to 1.4 m for the 30 cm sign, so the maximum range grows about 1.75x when the sign size doubles.
- The usable viewing angle shrinks with distance: the 30 cm sign reads up to 40° at 1.0 m but only 20° at 1.4 m.
- The 15 cm sign nearly fails at 0.4 m, probably because the code is too large or clipped in the frame.
- The mission reading distance (about 0.94 m, 30 cm sign) is well inside the reliable zone.

### D5: five trips (Nav2 drives to a room, QR confirms arrival)

| Time | Expected | Read | Outcome | Seconds |
|---|---|---|---|---|
| 21:18:02 | room_101 | room_101 | OK | 26.3 |
| 21:18:41 | room_102 | room_102 | OK | 39.5 |
| 21:19:24 | room_103 | room_103 | OK | 43.1 |
| 21:21:30 | room_104 | room_104 | OK | 125.8 |
| 21:22:14 | room_101 | room_101 | OK | 44.1 |

The 125.8 s trip to room 104 is an outlier.

### D7: wrong-room test (signs for rooms 101 and 102 swapped)

| Time | Expected | Read | Outcome | Sweeps |
|---|---|---|---|---|
| 02:32:38 | room_101 | room_102 | MISMATCH | 0 |
| 02:34:18 | room_102 | room_101 | MISMATCH | 0 |
| 03:13:35 | room_101 | room_102 | MISMATCH | 0 |

Navigation succeeded in all three; only the sign revealed the error. A map-based pose alone cannot detect a swapped sign.

### D8: unreadable sign and fallback (sweeps ±20°, ±40°, ±60°)

| Time | Room | Heading offset | Outcome | Sweeps | Seconds |
|---|---|---|---|---|---|
| 03:34:57 | room_103 | +40° | OK (recovered) | 4 | 82.1 |
| 03:41:16 | room_104 | -40° | OK (recovered) | 5 | 144.8 |
| 03:37:32 | room_103 | +90° | UNREADABLE (gave up) | 6 | 103.3 |
| 03:32:08 | room_103 | +40° | UNREADABLE | 6 | 53.6 |

- The same +40° setup recovered once and failed once, so recovery is not guaranteed.
- With the earlier ±30° sweep, all offset trials failed. The saved frames showed the robot arriving in the corridor a short distance from the goal, so the sign was too far from the arrival heading for a heading sweep to find it.
- Each extra sweep costs roughly 8 to 15 s. A zero-offset trip took about 49 s, a 4-sweep recovery about 82 s and a 5-sweep recovery about 145 s.

### D10: demo

```bash
python3 trip.py room_103 room_101 room_104
```

Expected: `OK`, `MISMATCH` (room 101 reads `room_102`), `OK`. Room 104 may need sweeps because of arrival drift in the far corridor.

---

## Findings

- Signs and the map answer different questions: the map says where the robot is, and the sign says whether it is the right place.
- Signs only work inside a limited range and angle (D4), so arrival pose matters.
- A heading sweep fixes heading errors, not position errors.
- Room 104, the farthest from the start, needed the most sweeps. This is a hypothesis from the saved frames, not yet confirmed against ground truth.

## Limitations and future work

- A read from a sign at a different door (for example, across the corridor) counts as a mismatch. Publishing bearing and distance with each read, and accepting only centred signs, would fix this.
- Use each sign read, whose distance and bearing `qr_node` already estimates, to correct the robot's pose.
- Compare AMCL's pose with Gazebo ground truth after each trip to confirm the arrival-error hypothesis.

## Reference

J. Lee, S. Y. Choi, D. Hanley, T. Bretl, "Comparative Study of Visual SLAM-Based Mobile Robot Localization Using Fiducial Markers," IROS 2023 Workshop on Closing the Loop on Localization, arXiv:2309.04441.

## Starting points

- [opencv/opencv](https://github.com/opencv/opencv): QR detection
- [ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv): `cv_bridge`
- [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2): navigation and Simple Commander
- [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations): simulation
