# G1 Gazebo

Test project controlling a Unitree G1 in Gazebo Harmonic

## Clone
Clone with submodules
```bash
git clone git@github.com:adamwhats/g1_gazebo.git -b main --recurse-submodules
```

## Build
Either run the devcontainer in vscode, or build with:
```bash
docker build -f .devcontainer/Dockerfile -t g1_gazebo .
```

## Setup

### For GUI and Teleop
1. Enable X11 forwarding (required for GUI):
```bash
xhost +local:
```

2. Find gamepad controller device (required for teleop):
```bash
sudo apt-get install -y evtest
evtest
# Note the event number (e.g., /dev/input/event19)
```

## Run

### IK Demo
Runs the simulation with position control of the arms to random world-space goal poses:

```bash
docker run -it --rm \
  --privileged \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -w /workspace \
  g1_gazebo \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source install/setup.bash && \
           ros2 launch g1_task ik.launch.py"
```

### Teleop Demo
Runs the simulation with gamepad teleoperation control:

```bash
# Replace event19 with the controller's event number from evtest
docker run -it --rm \
  --privileged \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -w /workspace \
  g1_gazebo \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source install/setup.bash && \
           ros2 launch g1_task teleop.launch.py device:=/dev/input/event19"
```

**Teleop Options:**
- `device:=/dev/input/eventXX` - Specify your gamepad device
- `gui:=true` - Show Gazebo GUI (default: false for first-person camera view only)
- `log_level:=debug` - Set logging level (info, debug, warn, error)

## Troubleshooting

### Missing third-party packages (aws_robomaker, kobuki_ros, etc.)

The third-party repos are imported via `vcs` during the Docker build. In the container run
   ```bash
   cd /workspace/src && vcs import < kobuki/thirdparty.repos
   colcon build --packages-up-to g1_task --symlink-install
   source install/setup.bash
   ```