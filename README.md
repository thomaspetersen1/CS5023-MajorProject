# CS5023-FinalProject

Check out the demo video here!  
[![Major Project Demo](https://img.youtube.com/vi/xta0YFV4XPM/0.jpg)](https://www.youtube.com/watch?v=xta0YFV4XPM)


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

## Build

From the workspace root:

```bash
colcon build --packages-select tour_guide --symlink-install
source install/setup.bash
```
Source `install/setup.bash` in every new terminal that runs `ros2 ...` against this workspace.

## Launch

### Tour Guide (main launch):

**Terminal 1, on the robot SSH session, start everything:**

```bash
ros2 launch tour_guide tour_guide.launch.py start_tour_nodes:=true
```

Wait ~30 seconds for Nav2 to come up and the tour nodes to start.

**In RViz (opens automatically):**

- Confirm the map is visible
- Click **2D Pose Estimate** and click+drag on the map to set the robot's starting position and orientation. AMCL needs this before navigation will work.

**Terminal 2, start the tour after setting the initial pose:**

```bash
ros2 service call /start_tour std_srvs/srv/Trigger
```

> Note: the service only exists once tour_executor finishes initializing (~30s after launch).
> Confirm it's up first: `ros2 service list | grep start_tour`

**Terminal 3, monitor tour progress:**

```bash
ros2 topic echo /tour_status
```

Expected state sequence: `IDLE` → `PLANNING` → `NAVIGATING` → `DWELLING` → repeat → `IDLE` (tour complete)

**Terminal 4, interactive landmark CLI for requesting stops mid-tour:**

```bash
ros2 run tour_guide tour_cli
```

REPL commands:

- `l` - list available landmarks, numbered
- `s` - show current tour status: state, current target, remaining, visited
- `a <name|#> ...` - queue one or more landmarks for visit, or just type a bare number/name
- `c` - cancel the active tour
- `h` - help
- `q` - quit

When you queue a new landmark, the executor cancels the current goal and re-runs TSP from the robot's live pose, so the closest unvisited landmark becomes the new target. That might end up being the one you just added. After each arrival it also re-runs TSP from the pose it just reached.

Override the landmarks file:

```bash
ros2 run tour_guide tour_cli --ros-args -p landmarks_file:=/abs/path/to/landmarks.yaml
```

**Override the dwell duration for a longer or shorter spin on arrival:**

```bash
ros2 run tour_guide tour_executor --ros-args -p dwell_seconds:=8.0
```

---

### rosbridge (WebSocket bridge for the web UI):

The lab machines won't let us apt-install `ros-jazzy-rosbridge-suite`, so we build it from source inside the workspace. You only need to do this once per clone:

```bash
# 1. Clone rosbridge_suite at a Jazzy-compatible tag into src/
git clone -b 3.2.0 https://github.com/RobotWebTools/rosbridge_suite.git src/rosbridge_suite

# 2. Make a venv that still sees system rclpy
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# 3. Install the runtime pip deps. pymongo provides the BSON
#    implementation rosbridge_library actually needs (the standalone
#    `bson` PyPI package is broken for rosbridge, see issue #198).
pip install -r requirements.txt

# 4. Build the workspace. BUILD_TESTING=OFF skips a missing
#    ament_cmake_mypy dep that's used only for tests.
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Every new terminal that runs `ros2 ...` against this workspace needs the same three sources:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

rosbridge_server now comes up automatically as part of `tour_guide.launch.py` on port 9090.

### Webapp (browser-side UI on the lab PC):

The webapp ships as a prebuilt static export in `web/static-bundle.tar.gz`. No Node toolchain needed.

```bash
mkdir -p ~/tour-webapp
tar -xzf web/static-bundle.tar.gz -C ~/tour-webapp
cd ~/tour-webapp && python3 -m http.server 3000
```

Open `http://localhost:3000` in firefox/chrome on the lab PC with rosbridge running. You should see a green "Connected" badge and all 6 named landmarks listed.

---

### Map Maker:

```bash
# Terminal 1, start SLAM and RViz
ros2 launch tour_guide map_maker.launch.py

# Terminal 2, drive the robot around
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# Terminal 3, once the map looks complete in RViz, save it
ros2 run nav2_map_server map_saver_cli \
    -f ~/projects/CS5023-MajorProject/src/tour_guide/maps/map1
```

In rviz, Add-> Map -> set topic to /map  
Change fixed frame to map

## Testing

The TSP heuristic in `tour_guide/tsp.py` has unit tests that don't need ROS or the robot. Run them from the workspace root:

```bash
python -m pytest src/tour_guide/test/test_tsp.py -v
```

The tests load `src/tour_guide/config/landmarks.yaml` and check `nearest_neighbor_order` against three starting poses. `None` covers the case where AMCL hasn't published a pose yet, `(0, 0)` is the map origin, and `(-3, -8)` puts us near the charging stations. If you edit `landmarks.yaml`, the expected orders in `test_tsp.py` need to be recomputed.

If you also want the lab's `flake8` / `pep257` / copyright checks, run them through colcon:

```bash
colcon test --packages-select tour_guide
colcon test-result --verbose
```
