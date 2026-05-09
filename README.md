# WidowX 250 S Cartesian Trajectory Generation

This repository provides a complete implementation of **Task Space Trajectory Generation** and **Inverse Kinematics (IK)** for the Trossen Robotics WidowX 250S manipulator. The project is simulated using the **MuJoCo** physics engine.

## 🚀 Project Goal
The primary objective is to execute a smooth pick-and-place task:
1. **Lift** a yellow block from its starting coordinates.
2. **Navigate** over a brown wall obstacle by utilizing a vertical via-point with orientation to clear the workspace effectively.
4. **Place** the block precisely on top of a green target box.

## 🛠️ Core Components

### 1. Quintic Polynomial Trajectory
Uses 5th-order polynomials to generate paths where position, velocity, and acceleration are all continuous. This prevents the "jerky" motion that can lead to physical robot damage or simulation instability.

### 2. SLERP (Spherical Linear Interpolation)
Orientation is handled via Quaternions. Unlike standard Euler interpolation, SLERP ensures a constant angular velocity and follows the shortest path on the unit sphere.

### 3. Stable Inverse Kinematics
The IK solver uses an **Axis-Angle (Skew-Symmetric)** error method. 

## ⚙️ Mathematical Highlights

### Quintic Formula
$$q(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 + a_5t^5$$

### Rotation Error for IK
To avoid numerical drift, the orientation error is derived from the skew-symmetric part of the rotation error matrix:
$$\\mathbf{e}_{ori} = \\frac{1}{2} (R R_{ref}^T - R_{ref} R^T)^\\vee$$

## 🛠️ Installation & Running

1. **Clone the repo:**
   ```bash
   git clone [https://github.com/your-username/WidowX-Trajectory-Gen.git](https://github.com/your-username/WidowX-Trajectory-Gen.git)
   cd WidowX-Trajectory-Gen

## Trajectory Concept

The trajectory is planned in Cartesian space, not directly in joint space.

- Position waypoints are defined as `X0_ref`, `Xvia_ref`, and `Xf_ref`.
- Orientation waypoints are defined directly as quaternions, for example `quat0`, `quat_via`, and `quat_f`.
- The mocap body named `reference` is updated every timestep with `data.mocap_pos` and `data.mocap_quat`.
- The inverse kinematics solver converts that moving pose into a 6-joint configuration for the robot.


