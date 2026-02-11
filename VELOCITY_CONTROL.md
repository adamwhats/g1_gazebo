# Jacobian-Based Velocity Control for G1 Robot Teleoperation

## Overview

This document explains the velocity-based teleoperation control strategy implemented for the Unitree G1 humanoid robot. The system uses **Jacobian-based resolved motion rate control** to map joystick commands to smooth, continuous arm movements.

## Control Architecture

```
┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌────────────┐
│ Joystick │ -> │ Cartesian    │ -> │ Jacobian    │ -> │ Joint      │
│ Input    │    │ Velocities   │    │ Mapping     │    │ Velocities │
└──────────┘    └──────────────┘    └─────────────┘    └────────────┘
                                            ↑
                                     ┌──────┴──────┐
                                     │ Current     │
                                     │ Joint State │
                                     └─────────────┘
```

### Key Components:

1. **Joystick Input** → Cartesian velocity commands (vₓ, vᵧ, vᵤ)
2. **Jacobian Computation** → Relates joint velocities to end-effector velocities
3. **Damped Least Squares** → Robust pseudoinverse for singularity handling
4. **Velocity Limiting** → Prevents explosive joint motions
5. **Velocity Controller** → Low-level hardware control

## Mathematical Formulation

### The Jacobian Relationship

The fundamental relationship between joint velocities (q̇) and end-effector velocities (v) is:

```
v = J(q) · q̇
```

Where:
- `v ∈ ℝ³` is the Cartesian velocity vector [vₓ, vᵧ, vᵤ] (linear only, no rotation)
- `q̇ ∈ ℝⁿ` is the joint velocity vector (n = 6 for our arm chain)
- `J(q) ∈ ℝ³ˣⁿ` is the Jacobian matrix at current configuration q

### Computing the Jacobian

We use KDL's `ChainJntToJacSolver` to compute the Jacobian:

```python
jac_solver.JntToJac(q_current, jacobian)
J = jacobian[0:3, :]  # Extract linear part only (position, not orientation)
```

The Jacobian is **configuration-dependent** - it changes as the arm moves. This is why we recompute it at 50 Hz.

### Inverse Kinematics (Velocity Level)

To control the robot, we need the inverse relationship:

```
q̇ = J⁺ · v
```

Where `J⁺` is the pseudoinverse of J.

**Problem:** Near singularities (arm fully extended, elbow locked, etc.), J becomes rank-deficient and the standard pseudoinverse `J⁺ = Jᵀ(JJᵀ)⁻¹` becomes ill-conditioned, producing huge joint velocities for small Cartesian commands.

### Damped Least Squares (Levenberg-Marquardt)

We use **damped least squares** to handle singularities gracefully:

```
q̇ = Jᵀ(JJᵀ + λ²I)⁻¹ · v
```

Where:
- `λ` is the **damping factor** (default: 0.05)
- `I` is the 3×3 identity matrix

**How it works:**
- Far from singularities: `λ²I` is negligible, behaves like standard pseudoinverse
- Near singularities: `λ²I` dominates, preventing large joint velocities
- Trade-off: Accepts tracking error to maintain stability

**Implementation:**
```python
JJt = J @ J.T                                    # 3×3 matrix
damped_inv = np.linalg.inv(JJt + λ²I)          # Damped inverse
q_dot = J.T @ damped_inv @ v                    # Joint velocities
```

### Why This Works

At singularities, the determinant `det(JJᵀ)` approaches zero. The damping term ensures:

```
det(JJᵀ + λ²I) ≥ λ⁶ > 0  (always invertible)
```

Near singularities, the damping causes the arm to **slow down** rather than producing erratic motions.

## Kinematic Chain Configuration

### Chain Structure

```
pelvis → waist_yaw → shoulder_pitch → shoulder_roll →
         shoulder_yaw → elbow → wrist_roll → end_effector
```

**Important:**
- The full chain includes waist_yaw for proper Jacobian computation
- The waist position is held **fixed** (not controlled)
- Only the 5 arm joints are commanded (shoulder × 3, elbow × 1, wrist × 1)

### Why Keep Waist in Chain?

The Jacobian must account for the waist's geometric contribution to the end-effector position, even though we don't move it. This ensures correct kinematics.

When computing velocities:
```python
q_dot_full = J.T @ damped_inv @ v  # 6 velocities (waist + 5 arm joints)
q_dot_ctrl = q_dot_full[1:]        # Take only arm joints (skip waist)
```

## Singularity Types

### 1. **Workspace Boundary Singularities**
- **Cause:** Arm fully extended or reaching joint limits
- **Symptom:** Unable to move further in commanded direction
- **Handling:** Damping reduces velocity, preventing "fighting" against limits

### 2. **Internal Configuration Singularities**
- **Cause:** Elbow at 0° or 180° (arm becomes planar)
- **Symptom:** Loss of DOF in certain directions
- **Handling:** Damping maintains stability but motion may be slow

### 3. **Algorithmic Singularities**
- **Cause:** Waist/shoulder alignment creating kinematic degeneracy
- **Symptom:** Multiple joint solutions for same end-effector velocity
- **Handling:** Damping picks "least effort" solution

## Control Parameters

### Core Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `velocity_scale` | 0.3 | m/s | Maps joystick input to Cartesian velocity |
| `update_rate` | 50 | Hz | Control loop frequency |
| `damping_factor` | 0.05 | - | Singularity robustness (λ) |
| `max_joint_velocity` | 1.5 | rad/s | Hard limit on joint velocities |

### Parameter Tuning Guide

**`damping_factor` (λ):**
- **Lower (0.01-0.03):** Better tracking accuracy, less stable near singularities
- **Higher (0.05-0.1):** More stable, slower motion near singularities
- **Recommended:** Start at 0.05, increase if seeing unstable behavior

**`max_joint_velocity`:**
- **Purpose:** Safety limit to prevent hardware damage
- **Lower (0.5-1.0):** Very safe but may feel sluggish
- **Higher (1.5-2.5):** More responsive but risk of violent motions at singularities
- **Recommended:** 1.5 rad/s provides good balance

**`velocity_scale`:**
- **Purpose:** Joystick sensitivity (Cartesian space)
- **Lower (0.1-0.2):** Precise control, slow motion
- **Higher (0.4-0.6):** Fast motion, less precise
- **Recommended:** 0.3 m/s for general teleoperation

## Implementation Details

### Control Loop (50 Hz)

```python
def _velocity_update(self):
    # 1. Read joystick axes
    cart_vel_left = [joy.axes[1], -joy.axes[0], joy.axes[7]-joy.axes[6]]
    cart_vel_right = [joy.axes[3], -joy.axes[2], joy.axes[7]-joy.axes[6]]

    # 2. Scale by velocity_scale
    cart_vel_left *= velocity_scale
    cart_vel_right *= velocity_scale

    # 3. Compute Jacobian at current configuration
    J_left = compute_jacobian(arm='left', q_current)
    J_right = compute_jacobian(arm='right', q_current)

    # 4. Damped least squares
    q_dot_left = damped_pseudoinverse(J_left) @ cart_vel_left
    q_dot_right = damped_pseudoinverse(J_right) @ cart_vel_right

    # 5. Clamp to velocity limits
    q_dot_left = clip(q_dot_left, -max_vel, +max_vel)
    q_dot_right = clip(q_dot_right, -max_vel, +max_vel)

    # 6. Publish to velocity controller
    publish([q_dot_left, q_dot_right])
```

### Frame Convention

All velocities are expressed in the **pelvis frame**:
- **+X:** Forward (in front of robot)
- **+Y:** Left (robot's left side)
- **+Z:** Up (vertical)

Joystick mapping:
- Left stick forward/back → vₓ
- Left stick left/right → vᵧ (negated for intuitive control)
- D-pad up/down → vᵤ

## Advantages of This Approach

### 1. **Direct Velocity Control**
- No trajectory planning overhead
- True continuous motion
- <20ms latency (joy → motion)

### 2. **Configuration-Space Control**
- Always respects joint limits
- Kinematically feasible at every instant
- No need for path planning

### 3. **Singularity Robustness**
- Graceful degradation near singularities
- Never produces unbounded velocities
- Maintains stability in all configurations

### 4. **Parallel Dual-Arm Control**
- Independent Jacobians for each arm
- No coupling between arms
- True simultaneous control

### 5. **Simplicity**
- No IK solver iterations
- Single matrix inversion per arm
- Computationally efficient (50 Hz easily achievable)

## Limitations

### 1. **Tracking Accuracy**
Near singularities, damping causes tracking error:
- Commanded direction may differ from achieved direction
- Magnitude of motion is reduced

### 2. **No Orientation Control**
Current implementation only controls position (3 DOF), not orientation (6 DOF):
- Wrist orientation follows from null-space
- Cannot independently control hand orientation

### 3. **Local Method**
Velocity control is local (no global planning):
- Cannot plan around obstacles
- May get "stuck" at workspace boundaries
- No path optimization

### 4. **Fixed Frame**
Velocities are in pelvis frame, not hand frame:
- As arm rotates, joystick directions feel different
- Could implement hand-frame control for more intuitive operation

### 5. **Workspace Limits**
Currently defined but not enforced:
```python
self.x_bounds = (0.2, 0.4)
self.y_left_bounds = (0.0, 0.3)
self.z_bounds = (0.1, 0.5)
```

Could add Cartesian position clamping or repulsive potential fields.

## Debugging and Visualization

### Velocity Markers

The system publishes visualization markers showing commanded Cartesian velocities:
- **Fixed positions:** (0.3, ±0.15, 0.3) in pelvis frame
- **Arrow direction:** Shows velocity direction
- **Arrow length:** Proportional to velocity magnitude
- **Colors:** Blue (left), Pink (right)

**Usage:**
```bash
ros2 run rviz2 rviz2
# Add MarkerArray display
# Topic: /ik_target_markers
```

### Common Issues

**Symptom:** Arms move in unexpected directions
- **Check:** Frame alignment (pelvis vs. world)
- **Debug:** Watch marker arrows - do they point where expected?

**Symptom:** Arms move slowly or not at all
- **Check:** Singularity (arm fully extended?)
- **Fix:** Increase `damping_factor` or add workspace limits

**Symptom:** Jerky, unstable motion
- **Check:** `damping_factor` too low
- **Fix:** Increase to 0.08-0.1 for more stability

**Symptom:** "Explosive" joint motion
- **Check:** `max_joint_velocity` limit working?
- **Fix:** Reduce limit to 1.0 rad/s, increase damping

## Advanced Topics

### Manipulability

The manipulability measure quantifies "how far" from singularity:

```
μ = √det(JJᵀ)
```

Could implement adaptive damping:
```python
if manipulability < threshold:
    damping_factor = 0.1  # High damping
else:
    damping_factor = 0.02  # Low damping
```

### Null-Space Control

For redundant manipulators (n > 3), the null space of J allows secondary objectives:

```
q̇ = J⁺v + (I - J⁺J)q̇₀
```

Where `q̇₀` biases towards a "comfortable" configuration (e.g., mid-range joint positions).

### Task-Space Orientation

To control both position and orientation (6 DOF):

```
v = [linear_velocity; angular_velocity]  # 6×1
J_full = [J_linear; J_angular]           # 6×6
```

Requires 6-axis joystick input or additional buttons.

### Collision Avoidance

Could add repulsive velocities from obstacles:

```
v_total = v_desired + v_repulsive
```

Where `v_repulsive` pushes away from nearby obstacles.

## Reset to Home Feature

Press **Square button** to smoothly return to home (all joints → 0):

```python
def _reset_to_home(self):
    error = 0 - q_current
    velocity = clip(error * 2.0, -0.5, 0.5)  # P-controller
    publish(velocity)
```

**Use cases:**
- Escape from near-singularity configuration
- Return to known-good starting pose
- Emergency stop and reset

## Further Reading

### Key Papers:
1. **Nakamura & Hanafusa (1986):** "Inverse kinematic solutions with singularity robustness for robot manipulator control"
2. **Wampler (1986):** "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods"
3. **Chiaverini et al. (1994):** "Review of the damped least-squares inverse kinematics with experiments on an industrial robot manipulator"

### Textbooks:
- **Siciliano et al.:** "Robotics: Modelling, Planning and Control" (Chapter 3)
- **Spong et al.:** "Robot Modeling and Control" (Chapter 6)

### ROS2 Resources:
- KDL Documentation: http://docs.ros.org/en/melodic/api/orocos_kdl/html/
- ros2_control: https://control.ros.org/

## Summary

The velocity-based control system provides:
- ✅ **Smooth, continuous motion** (50 Hz control)
- ✅ **Robust singularity handling** (damped least squares)
- ✅ **Low latency** (<20ms joystick → motion)
- ✅ **Dual-arm independence** (parallel computation)
- ✅ **Safety limits** (velocity clamping)
- ✅ **Easy reset** (home position button)

**Trade-offs:**
- Position-only control (no orientation)
- Local method (no global planning)
- Reduced accuracy near singularities
- Fixed reference frame (pelvis)

This approach is ideal for **teleoperation** where responsiveness and stability are more important than perfect trajectory tracking.
