---
title: Self-Balancing Reaction Wheel
date: 2025-01-07 12:00:00 -0500
categories: 
tags:
image: https://i.postimg.cc/XJXD9mNz/Reaction-Wheelw-Background.png
math: true
---

## Overview

I started this project to expand my knowledge of embedded system design. I began learning embedded C in 2022 and wanted a project to practice and apply my skills. 
This project was inspired by [The Cubli](https://www.youtube.com/watch?v=n_6p-1J551Y), though I chose to focus on balancing a single axis for my first project.

As a mechanical engineer, I have taken courses in control systems, and I saw this project as an excellent opportunity to apply some of that knowledge. 
While I have previously used Arduino in several college projects, I wanted to gain experience with a more industry-relevant microcontroller. 
I decided on the STM32 for its large support community and the extensive IDE.

In order to balance, this device relies on the conservation of angular momentum. The weighted flywheel is the key component of the design.
As the flywheel's speed changes, it generates a reaction torque that alters the orientation of the device. A brushless motor is coupled to the flywheel to control its speed. 
Similar reaction wheels are commonly used in spacecraft to control orientation.

All manufacturing files and source code for this project can be found in the GitHub repository [here](https://github.com/austynloehr/stm32_reaction_wheel).
Please note that, while this is a simple desk toy, the BOM cost is approximately $300. The biggest expenses are the PCB and the brushless motor controller. 
I approached this project as a hobby and learning experience, rather than prioritizing cost-effectiveness.

If you plan to recreate the device, I recommend using this project as a reference for your own design.

## Mechanical Design & Simulation

All 3D design work for this device was done in NX. While some components (such as the battery, motor, pulleys, etc.) were purchased, I designed and 3D printed many of the critical parts.

The first design challenge I encountered was sizing the motor and flywheel. My goal was to create a device that could jump up and balance on its own.
The first step to modelling any dynamic system is a free body diagram.

## PCB Design

![Render.png](https://i.postimg.cc/X7tF3R0j/Render.png)
_Figure 1. Balance Controller Rendering_

![Screenshot-from-2025-01-07-23-39-28.png](https://i.postimg.cc/RFX4Zjsp/Screenshot-from-2025-01-07-23-39-28.png)
_Figure 2. Balance Controller Schematic ([PDF](https://github.com/austynloehr/stm32_reaction_wheel/blob/main/Hardware/Balance_Controller/Schematic_Rev_B.pdf))_

### Background

I was originally planning on having all the electronics housed on a breadboard, but as I got more into the project, I saw an opportunity to also learn PCB design with KiCad. 
Thanks to [Phil's Lab](https://www.youtube.com/@PhilsLab) on YouTube, I was able to learn KiCad and the basics of designing a PCB for an STM32 MCU. His channel has a lot of great free content 
that covers many important aspects of PCB design, helping beginners get started with designing their own boards quickly. Before starting the design, I spent quite a bit of time doing my own 
research to ensure I understood critical design concepts. I wanted to make sure I was following best practices for signal integrity, such as proper component placement, selecting coupling capacitors, 
and managing return paths.

### Design Requirements

- Fits within designed chassis
- 12VDC input from battery
- Export 12VDC, 20A (fused) to VESC motor controller
- Provisions for emergency stop switch, cuts power to motor controller
- STM32 MCU
- MPU6050 IMU
- Controllable status LEDs
- CAN interface with 120 Ohm terminating resistor 
- UART interface

### Power Supply

The board needed to export 12V and provide both 3.3V and 5V for onboard peripherals. Due to the 12V, 20A export requirement, much of the board was essentially a copper pour to handle this load. 
For handling the emergency stop, I decided to use a 3-position SPDT switch. This would allow all devices to be turned on or off and provide an additional circuit for charging. 
The idea for the charging circuit was to provide a means to charge the battery without powering any onboard devices.

For the 3.3V and 5V supplies, I chose to use two [MPM3610GQV-P](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MPM3610GQV-Z)
switched DC-DC converters. I selected these converters because they were able to handle a wide range of input voltages and met both my voltage and current output requirements. 
Each converter can output up to 1.2A, which is more than sufficient for my application. These converters were able to step down the 12V battery voltage to the 3.3V–5V range without generating significant heat. 
Configuring the output voltage for these converters was straightforward; it simply required designing the appropriate voltage divider circuit according to the datasheet.

![Screenshot-from-2025-01-22-21-33-15.png](https://i.postimg.cc/bNQ68Lsw/Screenshot-from-2025-01-22-21-33-15.png)
_Figure 3. Power Supply Overview_

I chose to design a 4-layer PCB for this application. This allowed me to have one power supply layer, one ground layer, and two signal routing layers. The stack-up was as follows: signal 1, ground, power, signal 2. 
This design choice worked well for minimizing EMI and made routing power to all my components straightforward—all I had to do was drop a via to the power layer.
Having a dedicated power layer gave me a lot of space on the first layer for signal routing, allowing me to fit 90% of the traces on it. 

![Screenshot-from-2025-01-22-20-40-52.png](https://i.postimg.cc/FRDFk0Wt/Screenshot-from-2025-01-22-20-40-52.png)
_Figure 4. Power Supply Layer_

### Component Integration

After the power supply strategy was figured out, selecting components for the board was fairly easy. Since this was my first PCB design, I was surprised at how straightforward it was to integrate the remaining ICs. 
Component datasheets contained all the necessary information to design any supporting circuits and select the proper resistors and decoupling capacitors.

The remainder of the board design was mainly a packaging exercise. Placing all the components in optimal positions while maintaining best practices for component spacing, trace width, and trace spacing was definitely 
the most time-consuming part of the design.

### Areas For Improvement

The header labeled "Bluetooth" is designed to interface with an [HC-05](https://www.amazon.com/HiLetgo-Wireless-Bluetooth-Transceiver-Arduino/dp/B071YJG8DR) as a plug-in Bluetooth module, though 
this was not my original intent. The first revision of the board actually had an 
[RN4871](https://www.mouser.com/ProductDetail/Microchip-Technology/RN4871-I-RM140?qs=BJlw7L4Cy79OHrwBC2yoPQ%3D%3D&utm_id=8790913657&gad_source=1&gclid=Cj0KCQiA7se8BhCAARIsAKnF3rz_K3YjmF2MrAvp45t4IT7zQfu_NQ0ccjG3RbIa1I9dpqvn00wipusaAuxSEALw_wcB), 
an onboard UART bluetooth module.
Unfortunately, I was never able to get this module to work for my application. The goal was to log important signals over UART and wirelessly record the data on my computer.
I spent a few weeks working with the configuration but was never able to maintain a connection longer than a few seconds on both Linux and Windows machines. 
The easiest solution was to integrate the HC-05 module. I did most of my initial testing with this device, and I knew it would work if I were to pay for a second board revision to be manufactured. 

If my goal hadn't been to learn how to program in the STM32 ecosystem, I would have replaced both the MCU and Bluetooth module with an ESP32 or a similar SOC that met my interface requirements. 
If I had gone in this direction, I think I would have used Wi-Fi with an IoT protocol instead of Bluetooth. I believe that would have made it much easier to interface with a computer and view live data. Maybe a project for another day...

Oh and one last thing, the pinout on the charger connector is backwards from the normal convention. Never quite had the motivation to go back and fix that one.

## Software Architecture



## Angle Estimation

To design a balance control system, I first needed accurate feedback of the reaction wheel's current angle. The balance controller PCB includes an onboard IMU (MPU-6050) with both an accelerometer and a gyro. 
I used data from these sensors to estimate the reaction wheel's orientation.

### Accelerometer Estimate

Using the accelerometer, I was able to calculate the current angle using the gravity vector. Since the device only rotates around a single axis, the angle can be estimated using only the X and Y acceleration measurements.
If the device is perfectly balanced and unaffected by external forces, the gravity vector would align with the IMU’s Y-axis, and $$ a_y $$ would read around 9.8 m/s². As the device deviates from this position, more of the gravity
vector will be in the IMU's X-axis, and the magnitude of $$ a_x $$ would increase while $$ a_y $$ decreases. Using basic trigonometry, the device's angle can be estimated using equation \eqref{eq:theta_accel}. 

$$
\begin{equation}
  \theta = \tan^{-1}\left(\frac{a_x}{a_y}\right)
  \label{eq:theta_accel}
\end{equation}
$$

![Screenshot-from-2025-01-23-22-20-51.png](https://i.postimg.cc/28CR3DSs/Screenshot-from-2025-01-23-22-20-51.png)
_Figure 5. Accelerometer Angle Calculation_

This estimate is only accurate if there are no external 
net-forces, other than gravity, acting on the system. Any other net-forces will be seen in the accelerometer measurements, and throw off the angle estimate. This estimate is only accurate if there are no external net forces, 
other than gravity, acting on the system. Any other net forces will be detected by the accelerometer and will throw off the angle estimate. The advantage of this method is that it can always calculate an estimate of the 
device's orientation without needing information about its previous position.


### Gyro Estimate

Using the gyro, I was able to calculate the current angle by integrating the device's angular velocity from a known starting position. Assume the device is rotating at a constant 1000 deg/s and the initial angle of the device 
is known to be 45 degrees. If the device rotates for 5 ms, it will have rotated a total of 5 degrees, ending at a new angle of 50 degrees. This same concept applies, except that the angular velocity is not constant. 
This is not a problem, since the gyro can measure the device's angular velocity at any point in time. This concept is described by equation \eqref{eq:theta_gyro}.

$$
\begin{equation}
  \int_{t_0}^{t} \omega(t) \, dt
  \label{eq:theta_gyro}
\end{equation}
$$omega

Unfortunately, this continuous system cannot be perfectly modeled on an embedded system. However, it can be approximated using Euler's method for integration, as shown in equation \eqref{eq:theta_gyro_euler}.
In this method, the current angular velocity is multiplied by the sample time to calculate an angular displacement, which is then added to the previous angle to obtain the current angle.
While there are more accurate methods for discrete integration, this approach proved sufficient for my application. The estimation algorithm ran at 500 Hz, which helped minimize integration errors.


$$
\begin{equation}
  \theta_{n} = \theta_{n-1} + \Delta t \cdot \omega_n
  \label{eq:theta_gyro_euler}
\end{equation}
$$

![Screenshot-from-2025-01-23-22-30-26.png](https://i.postimg.cc/zv0ZyfGr/Screenshot-from-2025-02-01-21-44-14.png)
_Figure 6. Gyro Angle Calculation_

The main advantage of this estimation method is that it is less susceptible to external forces. However, there are some drawbacks: you must know the device's initial position, and the estimate will gradually drift over time. 
This drift occurs because sensor noise accumulates as it is integrated over time.


### Comparison 

The following plots show real data that was logged from the reaction wheel's IMU. The motor was spinning at a constant speed of approximately 5500RPM. The reaction wheel was manually flipped from one side (+45 degrees) to the other (-45 degrees).
At $$ t=16s $$, the reaction wheel motor was turned off to compare sensor noise. Data from the IMU was logged via bluetooth with a sample rate of 500Hz. Equations \eqref{eq:theta_accel} and \eqref{eq:theta_gyro_euler} 
were implimented in Python on my desktop. All angle estimation calculations and filtering were performed during post-processing.

Figure 7 shows the results of passing raw $$ a_x $$ and $$ a_y $$ values into equation \eqref{eq:theta_accel} to estimate the reaction wheel's orientation. Lines are drawn at $$ y = \pm 45^ \circ $$ for reference, marking the true positions
of the reaction wheel when flipped on its side. Note the drastic difference in noise when the motor is turned on versus off. This noise is the result of vibrations that are generated from the motor and propogated through the whole system.
I attempted to mitigate this issue by mounting the balance controller PCB on rubber isolators, but unfortunately, they were unable to provide sufficient damping.
While trends in the data are visible, the measurements are essentially unusable with the motor running. However, the estimates appear to be accurate when the motor is turned off.

![Screenshot-from-2025-02-01-19-12-53.png](https://i.postimg.cc/mgPBJ7mW/Screenshot-from-2025-02-01-19-12-53.png)
_Figure 7. Raw Accelerometer Angle Estimate_

Figure 8 shows the same data but $$ a_x $$ and $$ a_y $$ values were each passed through a 1st order digital low-pass filter (butterworth) before being passed into equation \eqref{eq:theta_accel}. Cutoff frequencies of 20Hz, 5Hz, and 1Hz are compared.
Each of these filtered examples represents a significant improvement over the raw data in Figure 7. Both the 20 Hz and 5 Hz filters still exhibit considerable steady-state noise and large angle spikes when the device falls on its side. 
Based on this data, the 1 Hz filter appears promising—at least for now.

![accel-filt-1.png](https://i.postimg.cc/kGShWXcZ/accel-filt-1.png)
_Figure 8. Accelerometer Filter Comparison_

Figure 9 contains the same data as Figure 8, but zoomed in on the timeframe $$ t=[15s, 17s] $$. This figure highlights the biggest disadvantage of additional filtering: response time. While the 1 Hz filter exhibits significantly less noise, 
it takes much longer to respond to changes in position. This increased response time is not acceptable for this application.

![accel-filt-2.png](https://i.postimg.cc/sXqNBZp7/accel-filt-2.png)
_Figure 9. Filter Response Time Comparison_

Figure 10 shows the results of passing $$ \omega_z $$ into equation \eqref{eq:theta_gyro_euler} with an initial condition of $$ 45^ \circ $$. Note that no additional filtering was performed on this data, and the signal is already much 
smoother compared to the accelerometer data. While the estimate initially starts off accurately, you can see it begin to drift around $$ t=10 $$ . Unfortunately, this drift is also not acceptable for this application.

![gyro.png](https://i.postimg.cc/7Z9t3R3x/gyro.png)
_Figure 10. Gyro Angle Estimate_

Figure 11 shows a comparison between gyro and filtered accelerometer angle estimates. If only there were a way to somehow combine these two estimates into a single, more accurate estimate...

![comparison.png](https://i.postimg.cc/nhpwQxQR/comparison.png)
_Figure 11. Accelerometer/Gyro Comparison_

### Kalman Filter

I’d like to preface this section by mentioning that the Kalman filter was not my first choice for the orientation estimation in this project. In fact, I initially tried to avoid designing a Kalman filter. 
I first explored using only the gyro with some integrator reset conditions, as well as a complementary filter. I initially achieved good results with the complementary filter, until I assembled the first prototype. 
However, the vibrations induced by the motor rendered both of these strategies unusable. This forced the project to be put on hold for several weeks while I learned about Kalman filter design. 

For anyone looking to learn about the topic without wading through formal proofs and intimidating matrix math, [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) is a fantastic resource. 
The author does an amazing job of breaking down concepts into layman's terms and providing real examples of how to implement the filters in Python.

#### State Matrix

$$
\begin{equation}
  \mathbf{\bar x} = \mathbf{F} \mathbf{x}
\end{equation}
$$

$$
\mathbf{\bar x} = \begin{bmatrix}
    \theta \\
    \dot{\theta} \\
    \ddot{\theta}
\end{bmatrix} = 

\begin{bmatrix}
    1 & \Delta t & \frac{1}{2} \Delta t^2 \\
    0 & 1 & \Delta t \\
    0 & 0 & 1
\end{bmatrix}

\begin{bmatrix}
    \theta \\
    \dot{\theta} \\
    \ddot{\theta}
\end{bmatrix}
$$

#### Process Noise Matrix

$$
\mathbf{Q} = \begin{bmatrix}
    \frac{\Delta t ^ 4}{4} & \frac{\Delta t ^ 3}{2} & \frac{\Delta t ^ 2}{2} \\
    \frac{\Delta t ^ 3}{2} & \Delta t ^ 2 & \Delta t \\
    \frac{\Delta t ^ 2}{2} & \Delta t & 1
\end{bmatrix} \sigma ^ 2
$$

#### Measurement Function

$$
\begin{equation}
  \mathbf{z} = \mathbf{H} \mathbf{x}
\end{equation}
$$

$$
\mathbf{z} = \begin{bmatrix}
    \theta \\
    \dot{\theta}
\end{bmatrix} =

\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 
\end{bmatrix}

\begin{bmatrix}
    \theta \\
    \dot{\theta} \\
    \ddot{\theta}
\end{bmatrix}

$$

#### Measurement Noise Matrix

$$
\mathbf{R} = \begin{bmatrix}
    \sigma_{\theta} ^ 2  & 0 \\
    0 & \sigma_{\dot{\theta}} ^ 2
\end{bmatrix}
$$

#### Initial Conditions

$$
\mathbf{X} = \begin{bmatrix}
    0 \\
    0 \\
    0
\end{bmatrix}
$$

$$
\mathbf{P} = \begin{bmatrix}
    45 & 0 & 0 \\
    0 & 4 & 0 \\
    0 & 0 & 4
\end{bmatrix}
$$

## Balance Control


## Data Logging & Visualization


## Final Remarks





