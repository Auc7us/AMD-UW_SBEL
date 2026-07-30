"""Project-local copy of lunar-manip/model/inverseKin.py.

The equations and link dimensions intentionally match the pulled tracked-builder
reference. Pass ``scale=2.0`` for the M113 builder and ``scale=1.0`` for the LRV.
"""

import numpy as np
from scipy.optimize import minimize


class RobotArmInverseKinematicsSolver:
    def __init__(self, scale=1.0):
        """Initialize the reference robot-arm inverse kinematics solver."""
        # Link lengths measured from the imported arm joint markers:
        # shoulder height, bicep, forearm, and wrist-to-gripper center.
        self.a1, self.a2, self.a3, self.a4 = (
            value * scale for value in (0.32516, 1.27, 1.143, 0.3577)
        )

    def forward_kinematics(self, theta):
        theta1, theta2, theta3, theta4 = theta
        return np.array(
            [
                self.f1(theta1, theta2, theta3, theta4),
                self.f2(theta1, theta2, theta3, theta4),
                self.f3(theta1, theta2, theta3, theta4),
            ]
        )

    def objective_function(self, theta, target_position):
        return np.linalg.norm(
            self.forward_kinematics(theta) - target_position
        )

    def f1(self, theta1, theta2, theta3, theta4):
        _a1, a2, a3, a4 = self.a1, self.a2, self.a3, self.a4
        s1, s2, s3, s4 = (
            np.sin(theta1),
            np.sin(theta2),
            np.sin(theta3),
            np.sin(theta4),
        )
        c1, c2, c3, c4 = (
            np.cos(theta1),
            np.cos(theta2),
            np.cos(theta3),
            np.cos(theta4),
        )
        sigma3 = c1 * c2 * c3 - c1 * s2 * s3
        sigma4 = c1 * c2 * s3 + c1 * c3 * s2
        return (
            a2 * c1 * c2
            + a4 * c4 * sigma3
            - a4 * s4 * sigma4
            - a3 * c1 * s2 * s3
            + a3 * c1 * c2 * c3
        )

    def f2(self, theta1, theta2, theta3, theta4):
        _a1, a2, a3, a4 = self.a1, self.a2, self.a3, self.a4
        s1, s2, s3, s4 = (
            np.sin(theta1),
            np.sin(theta2),
            np.sin(theta3),
            np.sin(theta4),
        )
        c1, c2, c3, c4 = (
            np.cos(theta1),
            np.cos(theta2),
            np.cos(theta3),
            np.cos(theta4),
        )
        sigma1 = c2 * c3 * s1 - s1 * s2 * s3
        sigma2 = c2 * s1 * s3 + c3 * s1 * s2
        return (
            a2 * c2 * s1
            + a4 * c4 * sigma1
            - a4 * s4 * sigma2
            - a3 * s1 * s2 * s3
            + a3 * c2 * c3 * s1
        )

    def f3(self, theta1, theta2, theta3, theta4):
        a1, a2, a3, a4 = self.a1, self.a2, self.a3, self.a4
        _s1, s2, s3, s4 = (
            np.sin(theta1),
            np.sin(theta2),
            np.sin(theta3),
            np.sin(theta4),
        )
        _c1, c2, c3, c4 = (
            np.cos(theta1),
            np.cos(theta2),
            np.cos(theta3),
            np.cos(theta4),
        )
        sigma5 = c2 * c3 - s2 * s3
        sigma6 = c2 * s3 + c3 * s2
        return (
            a1
            + a2 * s2
            + a3 * c2 * s3
            + a3 * c3 * s2
            + a4 * c4 * sigma6
            + a4 * s4 * sigma5
        )

    def inverse_kinematics_solver(
        self, target_position, tolerance=1e-3, elbow_up=False
    ):
        initial_guess = np.array(
            [
                np.arctan2(target_position[1], target_position[0]),
                np.pi / 2,
                -np.pi / 2,
                -np.pi / 2,
            ]
        )
        if elbow_up:
            target = np.asarray(target_position, dtype=float)

            def objective(theta):
                return self.objective_function(theta, target) + (
                    0.3 * max(0.0, theta[2]) ** 2
                )

            result = minimize(
                objective,
                initial_guess,
                method="BFGS",
                options={"gtol": 1e-3, "maxiter": 1000},
            )
        else:
            result = minimize(
                self.objective_function,
                initial_guess,
                args=(target_position,),
                method="BFGS",
                options={"gtol": 1e-3, "maxiter": 1000},
            )

        final_position = self.forward_kinematics(result.x)
        error = np.linalg.norm(final_position - target_position)
        if error <= tolerance:
            return result.x
        raise ValueError(
            "Inverse kinematics solver did not converge "
            f"(error={error}, message={result.message})"
        )
