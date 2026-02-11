# G1 Robot Teleoperation Project - Roadmap

This document tracks the major implementation steps for the G1 robot teleoperation system.

## Completed Steps

### 1. IK Service with Waist Toggle
**Date:** 2026-02-11

Implemented a launch parameter `enable_waist` to toggle the waist joint in IK calculations:
- Added `enable_waist` parameter to [ik.launch.py](src/g1_task/launch/ik.launch.py) (default: `true`)
- Modified [ik_service.py](src/g1_task/g1_task/ik_service.py) to conditionally include/exclude `waist_yaw_joint` from arm control
- When `enable_waist=false`, the waist remains fixed at its current position during IK solving
- This enables parallel dual-arm control without waist conflicts

**Usage:**
```bash
ros2 launch g1_task ik.launch.py enable_waist:=false  # For dual-arm control
ros2 launch g1_task ik.launch.py enable_waist:=true   # For single-arm control with waist
```

**Key Files:**
- [src/g1_task/launch/ik.launch.py](src/g1_task/launch/ik.launch.py) - Launch parameter definition
- [src/g1_task/g1_task/ik_service.py](src/g1_task/g1_task/ik_service.py) - IK solver implementation

### 2. Teleop Pose Target Generation
**Date:** Prior to 2026-02-11

Implemented teleop.py to generate pose targets for arm control:
- Controller input reading via Joy messages
- Pose target publishing for left and right arms

### 3. Teleop IK Integration & Servo Control
**Date:** 2026-02-11

Integrated IK service calls and trajectory execution into teleop:
- Added IK service client to [teleop.py](src/g1_task/g1_task/teleop.py)
- Added trajectory action client for `/arm_controller/follow_joint_trajectory`
- Implemented dual-arm IK requests at 10 Hz (configurable via `ik_request_rate` parameter)
- Pose updates run at 30 Hz, IK requests at 10 Hz for efficiency
- Asynchronous IK requests prevent blocking (one pending request per arm)
- Trajectories sent directly to arm controller as they arrive
- Uses `MultiThreadedExecutor` for concurrent service/action handling

**Key Features:**
- Simultaneous dual-arm control without conflicts (requires `enable_waist:=false`)
- Non-blocking IK requests with graceful failure handling
- Independent trajectory execution for each arm

**Configuration:**
- `update_rate`: Pose update frequency (default: 30 Hz)
- `ik_request_rate`: IK service request frequency (default: 10 Hz)
- `velocity_scale`: Joystick sensitivity (default: 0.3 m/s)

## Next Steps

### 4. Testing & Integration
Test the full teleop pipeline:
- Launch with `enable_waist:=false` for dual-arm control
- Verify smooth motion and collision-free trajectories
- Tune IK and trajectory parameters for responsiveness

### 5. Testing & Refinement
- Test dual-arm control in Gazebo simulation
- Tune trajectory parameters (duration, waypoints)
- Add collision avoidance if needed
- Performance optimization

## Technical Notes

### IK Chain Configuration
- **Base link:** pelvis
- **Left arm tip:** left_wrist_roll_rubber_hand
- **Right arm tip:** right_wrist_roll_rubber_hand
- **Full chain:** pelvis → waist_yaw → shoulder_pitch → shoulder_roll → shoulder_yaw → elbow → wrist_roll

### Design Decisions
- **Waist toggle:** When disabled, the IK solver still uses the full chain but locks the waist at its current position. This prevents conflicting waist commands from both arms while maintaining proper kinematics.
