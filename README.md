# 2-Link Robotic Manipulator Trajectory Tracking

This project implements the dynamic modeling and control of a planar 2-link robotic manipulator. The primary objective is to enable the robot's end-effector to track a predefined circular trajectory in the Cartesian plane using a Computed Torque Control (CTC) strategy.
##  Authors
1.) Ganipisetty Nischal
2.) Regalla Kowshikh
3.) Palla Rithik Reddy
4.) Chanda Akshay Kumar
5.) Gunda Yaswanth
6.) Sale Sangameshwar
##  Project Overview
Robotic manipulators require precise control to perform complex tasks. This simulation demonstrates a high-fidelity dynamic model and a non-linear control law designed to minimize tracking errors while handling the complex physics of multi-link systems.

### Key Features
* **Dynamic Modeling:** Derived the equations of motion using the Euler-Lagrange method, accounting for inertia, Coriolis forces, and gravity.
* **Computed Torque Control (CTC):** Implemented a non-linear control law that linearizes the robot dynamics to ensure global stability.
* **PD Feedback:** Integrated Proportional-Derivative (PD) gains ($K_p$ and $K_d$) to provide robust error correction during movement.
* **Simulink Integration:** Developed a modular block-diagram system to simulate real-time interaction between the controller and the plant.
* **Trajectory Generation:** Designed a circular path generator to test the manipulator's agility and accuracy.

##  Mathematical Model
The manipulator's dynamics are governed by the following equation of motion:

$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau$$



Where:
* **$M(q)$**: The $2 \times 2$ Inertia Matrix.
* **$C(q, \dot{q})$**: Centripetal and Coriolis vector.
* **$G(q)$**: Gravity vector.
* **$\tau$**: Applied joint torques.

The control law $\tau$ is designed as:
$$\tau = M(q)(\ddot{q}_d + K_p e + K_d \dot{e}) + C(q, \dot{q})\dot{q} + G(q)$$

##  System Parameters
The simulation uses the following physical constants for the links:

| Parameter | Link 1 | Link 2 |
| :--- | :--- | :--- |
| **Mass (m)** | 10 kg | 5 kg |
| **Length (l)** | 1 m | 0.8 m |

##  Results & Analysis
The simulation confirms that the end-effector accurately follows the desired circular path.



* **Error Convergence:** The tracking error rapidly converges to zero under the tuned PD gains.
* **Torque Optimization:** The controller successfully calculates the exact joint torques required to cancel out the non-linear dynamics of the arm.

## Repository Contents
* `/Simulink`: The `.slx` file containing the control loop and plant model.
* `/Scripts`: MATLAB scripts for initializing parameters and plotting tracking errors.
* `/Docs`: Detailed project statement and mathematical derivations.

---
*Developed for the Robotics and Automation evaluation.*
