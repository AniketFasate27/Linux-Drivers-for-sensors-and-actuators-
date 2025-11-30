# Linux-Drivers-for-sensors-and-actuators-

# ROS2 Linux Sensor & Actuator Drivers

Robust, modular **ROS 2 drivers** for common robotics **sensors and actuators** running on Linux-based platforms (Raspberry Pi, Jetson, x86).  
This package is designed to make it easy to bring real hardware into your ROS 2 ecosystem with clean interfaces and consistent topics.

---

## 🚀 Overview

This repository provides ROS 2 nodes for:

- **Sensors**
  - GPS (NMEA over UART/USB)
  - Ultrasonic distance sensors
  - IMUs (e.g., BNO055, LSM6DSOX + LIS3MDL)
  - Wheel encoders
  - 2D LiDAR (e.g., RPLIDAR or similar)
  - Environmental sensors (temperature / pressure)

- **Actuators**
  - DC motors (PWM + direction)
  - Servo motors
  - Relays / digital outputs
  - Status LEDs / buzzer

All drivers are implemented as **ROS 2 nodes** with parameterized configuration and standard message types to plug directly into higher-level stacks (navigation, localization, mapping, etc.).

---

## ✨ Key Features

- 🧩 **Modular design** – each sensor/actuator has a dedicated node
- 📡 **Standard ROS 2 messages** – `sensor_msgs`, `geometry_msgs`, etc.
- ⚙️ **Configurable via YAML** – ports, baud rates, pins, frames, filters
- 🧪 **Hardware-in-the-loop friendly** – easy to test with `ros2 topic echo`
- 🧱 **Linux-native I/O** – uses `/dev/tty*`, I²C/SPI, and GPIO libraries
- 📦 **Ready for integration** – works with `rviz2`, `nav2`, TF, and URDF models

---

## 🧱 Repository Structure (Example)

```text
.
├── gps_driver/           # GPS node: NMEA → NavSatFix, TwistStamped
├── imu_driver/           # IMU node: orientation, angular velocity, accel
├── ultrasonic_driver/    # Ultrasonic distance → Range
├── encoder_driver/       # Wheel encoder counts / velocity
├── lidar_driver/         # 2D LiDAR → LaserScan
├── motor_driver/         # DC motor control (cmd_vel or direct)
├── servo_driver/         # Servo PWM control
├── relay_driver/         # Digital IO / relays
├── config/               # YAML configs for all nodes
├── launch/               # Example bring-up launch files
└── README.md

