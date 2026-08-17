# Arduino-Based cross omniwheel configuration Robot Controller

Arduino-based control firmware for a joystick-driven robot with IMU-based PID stabilization and holonomic drive control.

This repository contains embedded firmware that integrates USB joystick input, gyroscope feedback from an MPU6050 IMU, and real-time PID control to achieve stable and responsive motion control for a four-motor robotic platform.

---

## Repository Contents

* **`ArduinoPID.ino`**: Main firmware implementing joystick input processing, yaw estimation, PID control, and motor mixing logic.
* **`.gitignore`**: Standard Git ignore configuration.

---

## System Overview

The firmware performs the following functions:

* Reads a USB joystick or game controller using the USB Host and HID stack.
* Acquires angular velocity data from an MPU6050 IMU via I2C.
* Estimates yaw angle by integrating gyroscope Z-axis data.
* Computes PID control output for yaw stabilization.
* Combines translational joystick input with rotational PID correction.
* Drives four motors in a holonomic configuration.
* Outputs diagnostic data over the serial interface.

---

## Hardware Requirements

* Arduino-compatible board with USB Host support (e.g., Arduino Due or Arduino with USB Host Shield)
* USB joystick or game controller
* MPU6050 IMU
* Four DC motors
* Cytron motor drivers (or compatible PWM/DIR drivers)
* External motor power supply
* Appropriate wiring and connectors

---

## Software Requirements

* Arduino IDE or Arduino CLI
* **Required libraries** (can be installed via the Arduino Library Manager or manually):
* USB Host library
* HID Universal
* USB Hub
* Joystick report parser
* MPU6050
* CytronMotorDriver
* Wire, SPI, and math libraries



---

## Control Architecture

### Joystick Input Processing

* Translational motion is derived from joystick X and Y axes.
* Rotational input is derived from trigger or rotational axes.
* Dead-zone filtering is applied to reduce noise.
* Joystick inputs are converted to polar coordinates for motion computation.

### Yaw Estimation

* Z-axis angular rate from the MPU6050 gyroscope is integrated over time.
* The time step ($dt$) is computed using microsecond-resolution timing.

### PID Control

Yaw stabilization is achieved using a PID controller:

* **Proportional (P)**: Reacts to instantaneous yaw error.
* **Integral (I)**: Compensates for steady-state offset.
* **Derivative (D)**: Improves damping and reduces overshoot.

The PID output contributes to the rotational component of motor commands.

### Motor Mixing (Holonomic Drive)

Motor velocities are computed using standard mecanum drive equations:

$$v_1 = \text{power} \cdot \sin(\theta - 45^\circ) + \text{rotation}$$

$$v_2 = \text{power} \cdot \sin(\theta + 45^\circ) - \text{rotation}$$

$$v_3 = \text{power} \cdot \sin(\theta - 45^\circ) - \text{rotation}$$

$$v_4 = \text{power} \cdot \sin(\theta + 45^\circ) + \text{rotation}$$

Where:

* $\theta$ is the joystick direction angle.
* $\text{power}$ is the joystick magnitude.
* $\text{rotation}$ includes PID-based yaw correction.

## Kinematic Model (Cross-Omni Drive)

The firmware calculates motor velocities using polar coordinates, where $p$ is the joystick magnitude (`power`) and $\theta$ is the joystick angle. These equations translate into a standard inverse kinematics matrix for a 45-degree X-drive configuration.

### 1. Polar to Cartesian
First, we define the Cartesian input vectors for the robot's target velocity:
*   $x = p \cos(\theta)$ (Lateral/Strafing motion)
*   $y = p \sin(\theta)$ (Forward/Backward motion)
*   $\omega$ = Rotational command (PID output and manual rotation)

### 2. Trigonometric Conversion
The base polar equations for the motors are:
$$v_1 = p \sin(\theta - 45^\circ) + \omega$$
$$v_2 = p \sin(\theta + 45^\circ) - \omega$$
$$v_3 = p \sin(\theta - 45^\circ) - \omega$$
$$v_4 = p \sin(\theta + 45^\circ) + \omega$$

Applying the angle sum/difference identities ($\sin(\theta \pm 45^\circ) = \sin(\theta)\cos(45^\circ) \pm \cos(\theta)\sin(45^\circ)$) and factoring out $\frac{\sqrt{2}}{2}$ yields:
$$p \sin(\theta \pm 45^\circ) = \frac{\sqrt{2}}{2} (p \sin(\theta) \pm p \cos(\theta))$$
$$p \sin(\theta \pm 45^\circ) = \frac{\sqrt{2}}{2} (y \pm x)$$

### 3. Cartesian Equations
Replacing the trigonometric functions with our Cartesian variables provides the linear algebraic equivalents:

$$v_1 = -\frac{\sqrt{2}}{2}x + \frac{\sqrt{2}}{2}y + \omega$$

$$v_2 = \frac{\sqrt{2}}{2}x + \frac{\sqrt{2}}{2}y - \omega$$

$$v_3 = -\frac{\sqrt{2}}{2}x + \frac{\sqrt{2}}{2}y - \omega$$

$$v_4 = \frac{\sqrt{2}}{2}x + \frac{\sqrt{2}}{2}y + \omega$$

### 4. The Inverse Kinematics Matrix
Extracting the coefficients forms the inverse kinematics matrix. This maps the desired chassis velocity vector $\begin{bmatrix} x & y & \omega \end{bmatrix}^T$ directly to the individual wheel velocities $\begin{bmatrix} v_1 & v_2 & v_3 & v_4 \end{bmatrix}^T$:

$$
\begin{bmatrix} v_1 \\ v_2 \\ v_3 \\ v_4 \end{bmatrix} = 
\begin{bmatrix} 
-\frac{\sqrt{2}}{2} & \frac{\sqrt{2}}{2} & 1 \\ 
\frac{\sqrt{2}}{2} & \frac{\sqrt{2}}{2} & -1 \\ 
-\frac{\sqrt{2}}{2} & \frac{\sqrt{2}}{2} & -1 \\ 
\frac{\sqrt{2}}{2} & \frac{\sqrt{2}}{2} & 1 
\end{bmatrix}
\begin{bmatrix} x \\ y \\ \omega \end{bmatrix}
$$

### Motor Pin Configuration

As defined in the firmware, pin assignments may be modified to match specific hardware configurations:

```cpp
CytronMD m1(PWM_DIR, 2, 22);
CytronMD m2(PWM_DIR, 4, 39);
CytronMD m3(PWM_DIR, 44, 23);
CytronMD m4(PWM_DIR, 13, 28);

```

---

## Serial Debug Output

* **Baud rate**: 115200
* **Output data includes**:
* Estimated yaw angle
* PID controller output



*Note: This information is intended for debugging and controller tuning.*

---

## Build and Upload Instructions

1. Open `ArduinoPID.ino` in the Arduino IDE.
2. Install all required libraries.
3. Select the correct board and serial port.
4. Upload the firmware.
5. Open the Serial Monitor at 115200 baud.

---

## PID Tuning Guidelines

1. Start with proportional gain ($K_p$) only.
2. Add integral gain ($K_i$) to reduce steady-state yaw error.
3. Introduce derivative gain ($K_d$) to improve damping.
4. Tune incrementally while observing system response.
