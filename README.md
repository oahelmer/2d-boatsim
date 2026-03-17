# Equations of Motion for Boat Simulation

The system is defined by the following state vector and equations:

## State Variables
The state `s` is an 8-dimensional vector:

\[
\mathbf{s} = [x,\; v_x,\; y,\; v_y,\; \theta,\; \omega,\; \phi,\; \dot{\phi}]^T
\]

Where:
- \(x, y\): Position coordinates
- \(v_x, v_y\): Linear velocities
- \(\theta\): Boat orientation (yaw angle)
- \(\omega = \dot{\theta}\): Angular velocity
- \(\phi\): Motor angle (relative to boat)
- \(\dot{\phi}\): Motor angular velocity

## Parameters
- \(M\): Mass of the boat
- \(I\): Moment of inertia of the boat
- \(I_{MOTOR}\): Moment of inertia of the motor
- \(L\): Length (moment arm) for motor force
- \(DAMPING\_V\): Linear velocity damping coefficient
- \(DAMPING\_W\): Angular velocity damping coefficient
- \(DAMPING\_PHI\): Motor angular velocity damping coefficient

## Inputs
- \(F\): Motor thrust force
- \(\tau\): Motor control torque

## Equations of Motion
The time derivative of the state \(\dot{\mathbf{s}} = f(\mathbf{s}, F, \tau)\) is given by:

\[
f(\mathbf{s}, F, \tau) = 
\begin{cases}
\dot{x} = v_x \\
\dot{v}_x = \frac{F \sin(\theta + \phi) - DAMPING\_V \cdot v_x}{M} \\
\dot{y} = v_y \\
\dot{v}_y = \frac{F \cos(\theta + \phi) - DAMPING\_V \cdot v_y}{M} \\
\dot{\theta} = \omega \\
\dot{\omega} = \frac{L \cdot F \sin(\phi) - DAMPING\_W \cdot \omega}{I} \\
\dot{\phi} = \dot{\phi} \\
\ddot{\phi} = \frac{\tau - DAMPING\_PHI \cdot \dot{\phi}}{I_{MOTOR}}
\end{cases}
\]