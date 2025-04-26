# 🤖 Robot Maze Solver Documentation

<div align="center">
  <img src="https://api.placeholder.com/400/300" alt="Robot Maze Solver" />
  <p><em>Two-wheel caster robot with ultrasonic sensors for autonomous maze navigation</em></p>
</div>

## 📋 Table of Contents

- [📊 Overview](#-overview)
- [🔧 Hardware Components](#-hardware-components)
- [🔌 Pin Configuration](#-pin-configuration)
- [💻 Software Architecture](#-software-architecture)
- [🧠 Maze Solving Algorithm](#-maze-solving-algorithm)
- [📍 Odometry & Position Tracking](#-odometry--position-tracking)
- [🌐 Web Interface](#-web-interface)
- [📡 Communication Protocol](#-communication-protocol)
- [🛠️ Setup Instructions](#️-setup-instructions)
- [📱 Usage Guide](#-usage-guide)
- [⚙️ Customization Options](#️-customization-options)
- [🔍 Troubleshooting](#-troubleshooting)
- [🚀 Future Enhancements](#-future-enhancements)

## 📊 Overview

This project implements an autonomous maze-solving robot based on the ESP32 microcontroller. The robot utilizes ultrasonic sensors for obstacle detection and encoders for precise movement tracking. It features a real-time web-based control interface that allows manual control and visualization of sensor data, position tracking, and motion parameters.

<div align="center">
  <img src="https://api.placeholder.com/500/160" alt="System Overview Diagram" />
</div>

## 🔧 Hardware Components

The robot is built with the following components:

| Component | Description | Purpose |
|:----------|:------------|:--------|
| <img src="https://api.placeholder.com/20/20" /> **ESP32** | Wi-Fi & Bluetooth Microcontroller | Main processing unit |
| <img src="https://api.placeholder.com/20/20" /> **DC Motors** | Two motors with encoders | Differential drive system |
| <img src="https://api.placeholder.com/20/20" /> **Ultrasonic Sensors** | 3× HC-SR04 (or similar) | Front, left, and right obstacle detection |
| <img src="https://api.placeholder.com/20/20" /> **Wheel Encoders** | 2× Rotary encoders | Position and velocity tracking |
| <img src="https://api.placeholder.com/20/20" /> **Power Supply** | Battery pack (not specified) | Power for electronics and motors |

## 🔌 Pin Configuration

<div align="center">
  <img src="https://api.placeholder.com/500/200" alt="Pin Configuration Diagram" />
</div>

| Component | Pin | Description |
|:----------|:----|:------------|
| **Left Motor** | 27 (IN1), 14 (IN2) | Controls left wheel movement |
| **Right Motor** | 12 (IN3), 13 (IN4) | Controls right wheel movement |
| **Left Encoder** | 34 (A), 35 (B) | Monitors left wheel rotation |
| **Right Encoder** | 32 (A), 33 (B) | Monitors right wheel rotation |
| **Front Ultrasonic** | 4 (TRIG), 2 (ECHO) | Detects obstacles in front |
| **Left Ultrasonic** | 25 (TRIG), 26 (ECHO) | Detects obstacles on left side |
| **Right Ultrasonic** | 18 (TRIG), 19 (ECHO) | Detects obstacles on right side |

## 💻 Software Architecture

The system consists of two main components:

<div align="center">
  <img src="https://api.placeholder.com/500/220" alt="Software Architecture Diagram" />
</div>

### 🧩 Arduino Code (ESP32)

The Arduino code implements these key functionalities:

- **📊 Sensor Reading**: Continuously reads data from ultrasonic sensors
- **⚙️ Encoder Tracking**: Uses interrupt-based counting for wheel rotation
- **🧭 Odometry Calculation**: Tracks robot position (X, Y) and orientation
- **🧠 Maze Solving Algorithm**: Implements right-wall following strategy
- **📡 WebSocket Server**: Communicates with the web interface
- **🚗 Motor Control**: Functions for movement (forward, backward, turns)

### 📱 Web Interface

The web interface provides:

- **📈 Real-time Data Visualization**: Position, motion, and sensor dashboards
- **🎮 Control Panel**: Manual and autonomous control buttons
- **📊 Live Charts**: Graphical representation of sensor and encoder data
- **🔍 Debug Information**: Raw JSON data display

## 🧠 Maze Solving Algorithm

The robot implements a right-wall following algorithm, which is effective for solving many maze types:

<div align="center">
  <img src="https://api.placeholder.com/500/220" alt="Maze Solving Algorithm Diagram" />
</div>

### 📝 Algorithm Logic

```
IF front_distance < FRONT_CLEARANCE OR right_distance < WALL_DISTANCE - 2:
    TURN LEFT
ELSE IF right_distance > WALL_DISTANCE + 10:
    TURN RIGHT (to find the wall)
ELSE IF right_distance > WALL_DISTANCE + 2:
    SLIGHT RIGHT (correction to stay near wall)
ELSE IF right_distance < WALL_DISTANCE - 2:
    SLIGHT LEFT (correction to avoid wall)
ELSE:
    GO FORWARD
```

### 🚦 Algorithm States

| State | Description | Action |
|:------|:------------|:-------|
| `forward` | Clear path ahead | Moving straight |
| `correcting_left` | Too close to right wall | Slight left adjustment |
| `correcting_right` | Too far from right wall | Slight right adjustment |
| `right` | Right corridor detected | 90° right turn |
| `left` | Dead end detected | 90° left turn |
| `following` | Normal wall following | Standard forward movement |

## 📍 Odometry & Position Tracking

The robot implements precise odometry to track its position in the maze:

<div align="center">
  <img src="https://api.placeholder.com/500/220" alt="Odometry Tracking Diagram" />
</div>

### 📊 Tracking Parameters

- **📌 Position (X,Y)**: Current coordinates in meters
- **🧭 Orientation (θ)**: Robot heading in degrees
- **📏 Distance**: Total distance traveled in meters
- **⚡ Velocity**: Linear (m/s) and angular (rad/s) speeds

### 🔢 Odometry Calculation

```cpp
// Convert encoder ticks to distances
leftWheelDist = (deltaLeftEncoder / ticksPerRevolution) * 2 * PI * wheelRadius;
rightWheelDist = (deltaRightEncoder / ticksPerRevolution) * 2 * PI * wheelRadius;

// Calculate robot movement
deltaDistance = (leftWheelDist + rightWheelDist) / 2.0;
deltaTheta = (rightWheelDist - leftWheelDist) / wheelBase;

// Update position
robotX += deltaDistance * cos(robotTheta + (deltaTheta / 2.0));
robotY += deltaDistance * sin(robotTheta + (deltaTheta / 2.0));
robotTheta += deltaTheta;
```

## 🌐 Web Interface

The web interface provides a comprehensive dashboard for robot control and monitoring:

<div align="center">
  <img src="https://api.placeholder.com/500/300" alt="Web Interface Screenshot" />
</div>

### 📊 Dashboard Panels

1. **📍 Position Tracking**
   - X/Y coordinates, orientation, total distance

2. **📈 Motion Data**
   - Wheel RPM, linear/angular velocity

3. **📡 Sensor Readings**
   - Ultrasonic distances, encoder counts

4. **🎮 Control Panel**
   - Direction buttons, maze solving controls

5. **🔍 Raw Data Display**
   - JSON data for debugging

6. **📉 Real-time Charts**
   - Sensor and encoder data visualization

## 📡 Communication Protocol

The system uses WebSockets for bidirectional real-time communication:

### 📤 ESP32 to Web Interface (JSON)

```json
{
  "frontDistance": 25,
  "leftDistance": 30,
  "rightDistance": 8,
  "leftEncoder": 145,
  "rightEncoder": 150,
  "leftRPM": 60,
  "rightRPM": 62,
  "posX": 0.5,
  "posY": 0.3,
  "orientation": 45.0,
  "distanceTraveled": 2.4,
  "velocity": {
    "linear": 0.2
  },
  "odometry": {
    "angular": 0.1
  },
  "mazeStatus": "following",
  "isWallFind": true
}
```

### 📥 Web Interface to ESP32 (Commands)

```json
{
  "command": "forward"
}
```

Available commands:
- `forward`, `backward`, `turnLeft`, `turnRight`, `stop`
- `startMaze`, `stopMaze`

## 🛠️ Setup Instructions

<div align="center">
  <img src="https://api.placeholder.com/500/220" alt="Setup Instructions Diagram" />
</div>

### 🔧 Hardware Assembly

1. Connect motors to pins according to the [Pin Configuration](#-pin-configuration)
2. Mount ultrasonic sensors at the front, left, and right sides
3. Attach wheel encoders to motor shafts
4. Connect power supply to ESP32 and motors

### 💻 Software Setup

1. Install required libraries:
   - `Wire.h`, `ESPmDNS.h`, `ESPAsyncWebServer.h`, `WebSocketsServer.h`, `ArduinoJson.h`

2. Upload the Arduino code to ESP32

3. The ESP32 will create a Wi-Fi access point:
   - **SSID**: `ONESTEP`
   - **Password**: None (open network)

### 🔗 Connecting to the Robot

1. Connect to the `ONESTEP` Wi-Fi network from your device
2. Open a web browser and navigate to `http://192.168.4.1`
3. The control interface should load automatically

## 📱 Usage Guide

<div align="center">
  <img src="https://api.placeholder.com/500/220" alt="Usage Guide Diagram" />
</div>

### 🎮 Manual Control

- Use the direction buttons to control the robot movement
- The dashboard will display real-time sensor readings and position

### 🤖 Autonomous Maze Solving

1. Place the robot next to a wall (preferably with wall on right side)
2. Click `startMaze` to begin autonomous navigation
3. Monitor the robot's status and sensor readings in real-time
4. Click `stopMaze` to halt autonomous operation

### 📊 Data Monitoring

- View position data in the top-left panel
- Monitor sensor readings in the top-right panel  
- Watch trend graphs at the bottom of the dashboard

## ⚙️ Customization Options

You can modify these parameters to adapt the robot to different environments:

### 📐 Physical Parameters

```cpp
float wheelRadius = 0.022;  // Wheel radius in meters (22mm)
float wheelBase = 0.125;    // Distance between wheels in meters (12.5cm)
const float ticksPerRevolution = 1655;  // Encoder ticks per wheel revolution
```

### 🧠 Maze Solving Parameters

```cpp
#define WALL_DISTANCE 6    // Ideal distance from wall in cm
#define MIN_DISTANCE 5     // Minimum distance to consider an obstacle  
#define FRONT_CLEARANCE 4  // Minimum front clearance to move forward
```

### 🖥️ Web Interface Customization

The web interface can be modified by editing the HTML file:
- Change colors and styles
- Add new visualization widgets
- Customize control buttons

## 🔍 Troubleshooting

| Problem | Possible Cause | Solution |
|:--------|:--------------|:---------|
| 🔴 **Robot Not Moving** | Motor connections incorrect | Check wiring and pin definitions |
|  | Insufficient power | Ensure adequate battery voltage |
|  | Code upload issues | Re-upload Arduino sketch |
| 🔴 **Incorrect Turns** | Encoder issues | Verify encoder readings and connections |
|  | Incorrect wheel parameters | Calibrate `wheelRadius` and `wheelBase` |
| 🔴 **Web Interface Not Loading** | Wi-Fi connection issues | Verify connection to ONESTEP network |
|  | Wrong IP address | Navigate to http://192.168.4.1 |
| 🔴 **Maze Solving Problems** | Sensor readings inaccurate | Calibrate ultrasonic sensors |
|  | Wall following parameters | Adjust WALL_DISTANCE value |

## 🚀 Future Enhancements

Potential improvements for the project:

1. **🔄 PID Control**: Implement PID control for more precise motor control
2. **🗺️ Maze Mapping**: Create and store a map of the explored maze
3. **🧮 Path Optimization**: Implement algorithm to find shortest path after mapping
4. **🔋 Battery Monitoring**: Add voltage monitoring for battery level display
5. **📱 Mobile App**: Develop a dedicated mobile application for control
6. **💾 Data Logging**: Add SD card logging for long-term data storage
7. **🌐 Cloud Connectivity**: Add IoT capabilities for remote monitoring

---

<div align="center">
  <p>📝 Documentation generated for Robot Maze Solver Project</p>
  <p>🏆 Created with ❤️ for robotics enthusiasts</p>
</div>