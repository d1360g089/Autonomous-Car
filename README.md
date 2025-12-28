# Autonomous RC Car
# Embedded Systems & Autonomous Control Project

This project is an Arduino-based RC car capable of both manual wireless control and basic autonomous navigation. The system integrates DC motors, an nRF24L01 wireless module, an ultrasonic distance sensor, and a BNO055 IMU to explore autonomy, sensor limitations, and control strategies on constrained embedded hardware.

The project focuses on systems integration, real-time control, and the practical challenges of implementing autonomy with noisy sensors and limited computational resources.

# Autonomous RC Car with IMU-Assisted Scanning and Obstacle Avoidance

The autonomous mode uses a forward-driving behavior combined with obstacle detection. When an obstacle is detected within a defined threshold, the vehicle stops and performs a 360° rotational scan using yaw data from the BNO055 IMU. Distance measurements are taken at fixed angular increments using an ultrasonic sensor. The system then selects the direction with the greatest clearance and rotates toward it before resuming forward motion.

Yaw estimation is derived from the IMU’s quaternion output, avoiding Euler-angle singularities and allowing smoother rotational tracking during scanning and turning.

# Control Strategy and Algorithm Design

The autonomous behavior is structured as a simple state-based system:

1. Forward Drive: Continuous forward motion while no obstacle is detected
2. Detection: Ultrasonic sensor triggers a stop when distance falls below a threshold
3. Scan: In-place rotation while recording distance measurements every ~10°
4. Decision: Selection of the direction with maximum measured clearance
5. Reorientation: IMU-based rotation to the selected heading
6. Resume Motion: Return to forward driving
Motor control is implemented using PWM-based differential drive. Turning is achieved by driving left and right wheels in opposite directions at equal speeds.

# IMU and Orientation Estimation

Sensor: BNO055 IMU

Orientation Representation: Quaternions

Usage: Yaw tracking for rotational scanning and angle-based turns

The IMU is used strictly for heading estimation, not full position tracking. While quaternion-based yaw computation improves rotational stability and avoids gimbal lock, long-term drift and noise remain limiting factors in the absence of encoder-based feedback or sensor fusion.

# Manual Wireless Control (RC Mode)

In manual mode, joystick inputs are transmitted wirelessly using an nRF24L01 module. The joystick’s X and Y axes are mapped to left and right motor speeds, enabling differential steering.

This mode served both as a functional remote-control system and as a baseline for testing motor response, communication latency, and drive behavior prior to autonomous operation.

# Key Challenges and Limitations

This project intentionally exposed several real-world constraints:

  Ultrasonic Sensor Noise: Inconsistent readings due to surface angle and environmental factors
  
  IMU Drift: Accumulated yaw error during extended scans
  
  Lack of Odometry: No wheel encoders, limiting position estimation
  
  Blocking Control Logic: Delays and blocking loops reduce responsiveness
  
  Compute Constraints: Arduino-class hardware limits real-time sensor fusion and control complexity
  
  These limitations significantly influenced system design choices and algorithm simplicity.

# Engineering Lessons Learned

IMU-only autonomy is insufficient for reliable navigation

Sensor noise dominates low-cost robotic systems

Control performance is tightly coupled to hardware constraints

Autonomous systems require perception, estimation, and control, not just decision logic

This project reinforced the importance of honest failure analysis and system-level thinking in embedded robotics.
Demo Videos

Links to short demonstrations showing:
(YouTube / TikTok links)

Manual RC control: https://www.tiktok.com/@typicalcircuit/video/7583457047086189855    
https://www.tiktok.com/@typicalcircuit/video/7581312163025931550

Autonomous obstacle avoidance: https://www.tiktok.com/@typicalcircuit/video/7578304480257576222
Extra Videos for the project journey 😊:
https://www.tiktok.com/@typicalcircuit/video/7572046296685530398
https://www.tiktok.com/@typicalcircuit/video/7576484275625725214
https://www.tiktok.com/@typicalcircuit/video/7577886115139439903
https://www.tiktok.com/@typicalcircuit/video/7578291330447707423
https://www.tiktok.com/@typicalcircuit/video/7578304480257576222

🧠 How It Works

Microcontroller: Arduino (UNO-class)
Motors: DC motors with H-bridge drivers
Wireless: nRF24L01 (2.4 GHz)
Sensors: BNO055 IMU (yaw estimation)  Ultrasonic sensor (distance measurement)
Control: PWM-based differential drive with state-based autonomy

# 📁 Files

receiver/Autonomous.ino

Autonomous + manual control firmware

transmitter/Transmitter.ino

Joystick-based wireless controller firmware

🛠️ Components Used

Arduino Uno

BNO055 IMU

nRF24L01 RF modules (Tx/Rx)

Ultrasonic distance sensor

DC motors + motor drivers

Joystick modules

Breadboard, wiring, chassis

