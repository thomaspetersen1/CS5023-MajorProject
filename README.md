# CS5023-FinalProject

## Setup:

### ssh into robot:
```bash
ssh student@<nameFromRobot>.cs.nor.ou.edu
```
### Desktop to robot terminal:
```bash
robot-setup.sh
```
enter the turtlebot's name
```bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=8
export ROS_DISCOVERY_SERVER:";;;;;;;;10.194.16.61:11811;"
export ROS_SUPER_CLIENT=True
ros2 daemon stop
ros2 daemon start
```
### Keyboard Teleop:
Run robot-setup script and follow instructions
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```
### Start/Stop LiDAR:
```bash
# start
ros2 service call /start_motor std_srvs/srv/Empty "{}"
# stop
ros2 service call /stop_motor std_srvs/srv/Empty "{}"
```

## Launch

### Map Maker:
```bash
# Terminal 1 — start SLAM + RViz
ros2 launch tour_guide map_maker.launch.py

# Terminal 2 — drive the robot around
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# Terminal 3 — once the map looks complete in RViz, save it
ros2 run nav2_map_server map_saver_cli \
    -f ~/projects/CS5023-MajorProject/src/tour_guide/maps/map1
```

In rviz, Add-> Map -> set topic to /map   
Change fixed frame to map


### landmarks:
Landmark 1: Door by computer
At time 1777943503.597365803
- Translation: [-2.002, -3.760, -0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.003, 0.051, 0.999]
- Rotation: in RPY (radian) [0.001, 0.006, 0.102]
- Rotation: in RPY (degree) [0.055, 0.324, 5.864]
- Matrix:
  0.995 -0.102  0.006 -2.002
  0.102  0.995 -0.000 -3.760
 -0.006  0.001  1.000 -0.000
  0.000  0.000  0.000  1.000

Landmark 2: Pillar/Whiteboard
At time 1777943551.598660333
- Translation: [-6.389, -4.927, -0.001]
- Rotation: in Quaternion (xyzw) [-0.002, -0.000, 0.959, -0.284]
- Rotation: in RPY (radian) [0.001, 0.004, -2.566]
- Rotation: in RPY (degree) [0.031, 0.254, -147.026]
- Matrix:
 -0.839  0.544 -0.004 -6.389
 -0.544 -0.839 -0.002 -4.927
 -0.004  0.001  1.000 -0.001
  0.000  0.000  0.000  1.000

Landmark 3: Corner of cardboard box
At time 1777943594.594534581
- Translation: [-4.860, -5.134, -0.000]
- Rotation: in Quaternion (xyzw) [0.001, 0.000, 0.031, 1.000]
- Rotation: in RPY (radian) [0.003, 0.001, 0.062]
- Rotation: in RPY (degree) [0.156, 0.050, 3.543]
- Matrix:
  0.998 -0.062  0.001 -4.860
  0.062  0.998 -0.003 -5.134
 -0.001  0.003  1.000 -0.000
  0.000  0.000  0.000  1.000

Landmark 4: Under Desk
At time 1777943635.602329408
- Translation: [-5.270, -6.917, -0.001]
- Rotation: in Quaternion (xyzw) [-0.005, -0.001, 0.997, -0.071]
- Rotation: in RPY (radian) [-0.001, 0.010, -3.000]
- Rotation: in RPY (degree) [-0.031, 0.550, -171.888]
- Matrix:
 -0.990  0.141 -0.009 -5.270
 -0.141 -0.990 -0.002 -6.917
 -0.010 -0.001  1.000 -0.001
  0.000  0.000  0.000  1.000

Landmark 5: By the charging stations in middle of room
At time 1777943790.598298220
- Translation: [-2.714, -7.772, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.329, 0.944]
- Rotation: in RPY (radian) [0.000, 0.000, -0.669]
- Rotation: in RPY (degree) [0.021, 0.015, -38.359]
- Matrix:
  0.784  0.621 -0.000 -2.714
 -0.621  0.784 -0.000 -7.772
 -0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000


Landmark 6: Deep inside the cardboard room
At time 1777943714.595354116
- Translation: [-3.930, -3.416, -0.000]
- Rotation: in Quaternion (xyzw) [-0.000, 0.002, 0.698, 0.716]
- Rotation: in RPY (radian) [0.002, 0.003, 1.546]
- Rotation: in RPY (degree) [0.105, 0.182, 88.583]
- Matrix:
  0.025 -1.000  0.002 -3.930
  1.000  0.025  0.003 -3.416
 -0.003  0.002  1.000 -0.000
  0.000  0.000  0.000  1.000
