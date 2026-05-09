import numpy as np
import utility as ram
from forward_kinematics import forward_kinematics


def inverse_kinematics(q,X_ref):
    X_ref = np.asarray(X_ref).reshape(-1)

    if X_ref.size == 6:
        x_ref, y_ref, z_ref, phi_ref, theta_ref, psi_ref = X_ref
        R_ref = ram.euler2rotation(np.array([phi_ref, theta_ref, psi_ref]))
    elif X_ref.size == 7:
        x_ref, y_ref, z_ref = X_ref[:3]
        quat_ref = ram.quat_normalize(X_ref[3:7])
        R_ref = ram.quat2rotation(quat_ref)
    else:
        raise ValueError("X_ref must contain either 6 values [pos,euler] or 7 values [pos,quat].")

    # 1. First, compute forward kinematics to get the current state
    sol,robot = forward_kinematics(q)
    py_end_eff_pos = sol.end_eff_pos
    py_end_eff_rot = sol.end_eff_rot
    py_end_eff_quat = ram.rotation2quat(py_end_eff_rot)
    py_end_eff_euler = ram.rotation2euler(py_end_eff_rot)

    pos_err = py_end_eff_pos - np.array([x_ref, y_ref, z_ref])

    # Relative rotation error; zero when the reference and current orientation match.
    RRt = py_end_eff_rot @ R_ref.T
    eR = 0.5 * np.array([
    RRt[2, 1] - RRt[1, 2],
    RRt[0, 2] - RRt[2, 0],
    RRt[1, 0] - RRt[0, 1]
    ])
    return pos_err[0], pos_err[1], pos_err[2], eR[0], eR[1], eR[2]