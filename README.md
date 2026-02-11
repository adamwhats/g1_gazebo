# G1 Gazebo

Test project controlling a Unitree G1 in Gazebo Harmonic

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

2. Find your gamepad controller device (required for teleop):
```bash
# List input devices to find your controller
evtest
# Note the event number (e.g., /dev/input/event24)
```

## Run

### IK Demo (Inverse Kinematics with Task Sequencer)
Runs the simulation with automated IK-based pick and place tasks:

```bash
docker run -it --rm \
  --privileged \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace \
  -w /workspace \
  g1_gazebo \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source install/setup.bash && \
           ros2 launch g1_task ik.launch.py"
```

### Teleop Demo (Gamepad Control)
Runs the simulation with gamepad teleoperation control:

```bash
# Replace event24 with your controller's event number from evtest
docker run -it --rm \
  --privileged \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace \
  -w /workspace \
  --device=/dev/input/event24 \
  g1_gazebo \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source install/setup.bash && \
           ros2 launch g1_task teleop.launch.py device:=/dev/input/event24"
```

**Teleop Options:**
- `device:=/dev/input/eventXX` - Specify your gamepad device
- `gui:=true` - Show Gazebo GUI (default: false for first-person camera view only)
- `log_level:=debug` - Set logging level (info, debug, warn, error)

Example with GUI enabled:
```bash
docker run -it --rm \
  --privileged \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace \
  -w /workspace \
  --device=/dev/input/event24 \
  g1_gazebo \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source install/setup.bash && \
           ros2 launch g1_task teleop.launch.py device:=/dev/input/event24 gui:=true"
```