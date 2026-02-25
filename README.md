# Image-to-Edge-Robotic-Drawing-Arm

This project presents a complete digital-to-physical robotic drawing system that converts an input image into an edge map and reproduces it on paper using a 2-DOF planar robotic arm.

The system integrates:

Computer Vision

Inverse Kinematics

Embedded PWM Motor Control

3D Printed Mechanical Design

The platform is built using Python and executed on a Raspberry Pi.
Edge detection is implemented using OpenCV, while numerical computation relies on NumPy. Visualization is handled with Matplotlib.

The robotic arm is actuated using two SG90 Micro Servo motors and consists of fully 3D-printed components (STL files included).

System Architecture
Software Pipeline

Image selection (GUI)

Grayscale conversion & histogram equalization

Sobel edge detection

Binary thresholding

Morphological filtering

Contour extraction

Scaling to paper dimensions (A3 / A4)

Inverse kinematics computation

Real-time servo actuation

Hardware Components

Raspberry Pi

2 × SG90 Micro Servo

External 5V power supply (recommended)

3D-printed arm components (STL files in /mechanical)

Pen holder end-effector

Robotic Model

Type: 2-DOF Planar Articulated Arm

Link Lengths:

L1 = 200 mm

L2 = 200 mm

Control: PWM @ 50 Hz

Angular range: 0°–180°

Mathematical Model


Inverse kinematics for target point (x, y):

𝑟 = (𝑥^2 + 𝑦^2)^0.5
​

cos𝜃2 = (𝑥^2 + 𝑦^2 − 𝐿1^2 − 𝐿2^2) / 2 * 𝐿1 * 𝐿2


𝜃1 = tan^−1 * (𝑦/𝑥) − tan^−1 * ((𝐿2*sin𝜃2) / (𝐿1 + 𝐿2 * cos𝜃2))

Servo duty cycle:

𝐷𝑢𝑡𝑦 = (𝜃 / 18) + 2

Features

Manual zero-position calibration

Real-time drawing visualization

Contour scaling for A3 and A4

Edge-based artistic rendering

Debug simulation mode (without servos)

No Z-axis control (pen always in contact)

No trajectory optimization

Servo backlash affects precision

Limited torque due to micro servos

Future Improvements

Add third servo for pen lift (Z-axis)

Implement trajectory smoothing (Bezier interpolation)

Closed-loop control with encoders

Replace servos with stepper motors

Add workspace auto-scaling algorithm

Convert to G-code compatible format
