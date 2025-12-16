🤖 Robotics

Arduino-based control firmware for a joystick-driven robot with IMU-based PID stabilization and holonomic (mecanum) drive control.

This repository contains embedded firmware that integrates USB joystick input, gyroscope feedback from an MPU6050 IMU, and real-time PID control to achieve stable and responsive motion control for a four-motor robotic platform.


📁 Repository Contents

ArduinoPID.ino
Main firmware implementing joystick input processing, yaw estimation, PID control, and motor mixing logic.

.gitignore
Standard Git ignore configuration.


🧠 System Overview

The firmware performs the following functions:

🎮 Reads a USB joystick or game controller using the USB Host and HID stack

🧭 Acquires angular velocity data from an MPU6050 IMU via I2C

📐 Estimates yaw angle by integrating gyroscope Z-axis data

🔁 Computes PID control output for yaw stabilization

🔀 Combines translational joystick input with rotational PID correction

⚙️ Drives four motors in a holonomic (mecanum) configuration

📊 Outputs diagnostic data over the serial interface


🧰 Hardware Requirements

Arduino-compatible board with USB Host support
(e.g., Arduino Due or Arduino with USB Host Shield)

USB joystick or game controller 🎮

MPU6050 IMU 🧭

Four DC motors ⚙️

Cytron motor drivers (or compatible PWM/DIR drivers)

External motor power supply 🔋

Appropriate wiring and connectors


💻 Software Requirements

Arduino IDE or Arduino CLI

Required libraries (as used in the firmware):

USB Host library

HID Universal

USB Hub

Joystick report parser

MPU6050

CytronMotorDriver

Wire, SPI, and math libraries

Libraries can be installed via the Arduino Library Manager or manually.


🧩 Control Architecture
🎮 Joystick Input Processing

Translational motion is derived from joystick X and Y axes

Rotational input is derived from trigger or rotational axes

Dead-zone filtering is applied to reduce noise

Joystick inputs are converted to polar coordinates for motion computation


🧭 Yaw Estimation

Z-axis angular rate from the MPU6050 gyroscope is integrated over time

The time step (dt) is computed using microsecond-resolution timing


🔁 PID Control

Yaw stabilization is achieved using a PID controller:

Proportional (P): reacts to instantaneous yaw error

Integral (I): compensates for steady-state offset

Derivative (D): improves damping and reduces overshoot

The PID output contributes to the rotational component of motor commands.


🔀 Motor Mixing (Holonomic Drive)

Motor velocities are computed using standard mecanum drive equations:

v1 = power · sin(θ − 45°) + rotation
v2 = power · sin(θ + 45°) − rotation
v3 = power · sin(θ − 45°) − rotation
v4 = power · sin(θ + 45°) + rotation


Where:

θ is the joystick direction angle

power is the joystick magnitude

rotation includes PID-based yaw correction

⚙️ Motor Pin Configuration

As defined in the firmware:

CytronMD m1(PWM_DIR, 2, 22);
CytronMD m2(PWM_DIR, 4, 39);
CytronMD m3(PWM_DIR, 44, 23);
CytronMD m4(PWM_DIR, 13, 28);


Pin assignments may be modified to match specific hardware configurations.

📟 Serial Debug Output

Baud rate: 115200

Output data includes:

Estimated yaw angle

PID controller output

This information is intended for debugging and controller tuning.


🚀 Build and Upload Instructions

Open ArduinoPID.ino in the Arduino IDE

Install all required libraries

Select the correct board and serial port

Upload the firmware

Open the Serial Monitor at 115200 baud


📈 PID Tuning Guidelines

Start with proportional gain (Kp) only

Add integral gain (Ki) to reduce steady-state yaw error

Introduce derivative gain (Kd) to improve damping

Tune incrementally while observing system response
