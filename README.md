# 6-DOF-Robot-Inverse-Kinematics-IK-Solver

# Numerical Inverse Kinematics Solver for 6-DOF Serial Manipulators

![Version](https://img.shields.io/badge/Version-1.0.0-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Algorithm](https://img.shields.io/badge/Algorithm-Jacobian%20Damped%20Least%20Squares-orange)
![Platform](https://img.shields.io/badge/Platform-Web%20Assembly%20%2F%20JS-yellow)

## 📖 Abstract

This repository hosts a robust, web-based computational tool designed to solve **Inverse Kinematics (IK)** for 6-Degree-of-Freedom (6-DOF) robotic arms. Unlike analytical solvers that are restricted to specific robot geometries (e.g., those with spherical wrists), this solver utilizes a **Numerical Iterative Approach** based on the **Jacobian Damped Least Squares (DLS)** method. This ensures universality and robustness against kinematic singularities, making it applicable to a wide range of industrial manipulators (e.g., KUKA, ABB, Fanuc, Universal Robots).

The system allows for dynamic configuration of **Denavit-Hartenberg (DH)** parameters, supports real-time matrix verification, and implements **Automatic Random Restart** logic to mitigate local minima convergence issues.

##  Technical Specifications

* **Core Algorithm:** Damped Least Squares (DLS) / Levenberg-Marquardt approximation.
* **Kinematic Convention:** Supports both **Standard** and **Modified** Denavit-Hartenberg (DH) parameters.
* **Rotation Formalism:** Euler Angles (configurable sequence, optimized for Industrial Standard XYZ).
* **Convergence Strategy:** Multi-start iterative optimization to handle non-convex solution spaces.
* **Singularity Handling:** $\lambda$-damping factor integration near singular configurations.

##  Key Features

* **Universal Parametric Input:** Users can define custom link lengths ($a, d$) and twist angles ($\alpha$) to model any serial chain manipulator.
* **Real-time Error Quantization:** Displays the Euclidean distance error and rotational error matrix between the target and achieved end-effector poses.
* **Joint Space Constraints:** Implements rigorous clamping for physical joint limits ($Min/Max$ angles).
* **Matrix Verification Module:** detailed breakdown of Target Transformation Matrix ($T_{target}$), Achieved Matrix ($T_{curr}$), and the Difference Matrix ($T_{diff}$) for validation.

##  Mathematical Framework

### 1. Forward Kinematics (FK)
The end-effector pose is derived by multiplying homogeneous transformation matrices for each link $i$:
$${}^{0}T_{n} = \prod_{i=1}^{n} {}^{i-1}A_{i}(q_i)$$
Where ${}^{i-1}A_{i}$ is the transformation matrix derived from DH parameters.

### 2. Inverse Kinematics (Numerical)
The solver seeks to find the joint vector $\mathbf{q}$ that minimizes the error between current pose $\mathbf{x}_{curr}$ and target pose $\mathbf{x}_{target}$.
The differential relationship is given by:
$$\dot{\mathbf{x}} = J(\mathbf{q}) \dot{\mathbf{q}}$$

### 3. Damped Least Squares (DLS)
To invert the Jacobian matrix $J$ robustly, especially near singularities (where $\det(J) \approx 0$), the DLS method is employed:
$$\Delta \mathbf{q} = J^T (J J^T + \lambda^2 I)^{-1} \mathbf{e}$$
* $\lambda$: Damping factor (Non-zero to ensure invertibility).
* $\mathbf{e}$: Error vector (Position + Orientation).

## Comparative Analysis: Solver vs. RoboDK

When validating results against industry-standard simulation software like **RoboDK**, discrepancies in joint angles may be observed. This is mathematically expected due to **Kinematic Redundancy**.

| Phenomenon | Explanation |
| :--- | :--- |
| **Multiple Solutions** | A 6-DOF manipulator can reach a specific Cartesian pose in up to **8 distinct configurations** (e.g., Elbow-Up vs. Elbow-Down). Both solutions satisfy the kinematic equations ($Error \approx 0$). |
| **Local Minima** | Numerical solvers converge to the solution closest to the **Initial Guess**. RoboDK may utilize a different initialization vector. |
| **Euler Sequence** | This solver utilizes the **Rx-Ry-Rz (Extrinsic)** rotation order to align with generic industrial standards used in simulation environments. |

**Validation Metric:** The primary indicator of accuracy is the **Difference Matrix**. If elements of $T_{diff} \to 0$, the solution is valid regardless of joint configuration differences.


##  Contribution & Research

This project is intended for educational and research purposes in the field of **Robotic Control Systems** and **Automation**.
Contributions regarding **Analytical Solution integration** or **Trajectory Planning** are welcome.

---

**Author:** Arafat
*Mechanical Engineering Student | Automation & Robotics Enthusiast*
