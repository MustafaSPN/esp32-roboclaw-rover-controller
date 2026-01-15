# 🤖 ESP32 RoboClaw Rover Controller

This firmware is developed for the **Pathfinder** autonomous mobile robot. It serves as the low-level hardware interface, bridging the **ROS 2 Humble** network with the **RoboClaw** motor controller via **micro-ROS**.

![Micro-ROS](https://img.shields.io/badge/Micro--ROS-Humble-blue) ![ESP32](https://img.shields.io/badge/Board-ESP32-green) ![License](https://img.shields.io/badge/License-MIT-yellow)

## 📁 Firmware Variants

This repository contains three firmware variants for different use cases:

| Variant | Description | Motor Drivers | Odometry |
| :--- | :--- | :--- | :--- |
| **rover_controller_2wd** | 2-wheel drive with full odometry | 1x RoboClaw (0x80) | Position + Velocity |
| **rover_controller_4wd** | 4-wheel drive with velocity odometry | 2x RoboClaw (0x80, 0x81) | Velocity only (vx, vyaw) |
| **rover_controller_demo** | WiFi web-controlled demo (no ROS) | 2x RoboClaw (0x80, 0x81) | None |

## ⚠️ Critical Requirements

### 1. Board Package Version
**You MUST use ESP32 Board Package version `2.0.5`.**

Newer versions (2.0.6+) introduce breaking changes to the pre-compiled micro-ROS libraries, causing compilation errors or connection failures.

* **Arduino IDE:** Tools > Board > Boards Manager > Select version **2.0.5**.
* **PlatformIO:** Use `platform = espressif32@5.2.0` in your `platformio.ini`.

### 2. Motor Tuning
**Before using this firmware, you MUST tune your RoboClaw.**

This code sends **Velocity Commands** (QPPS). If the internal PID of the RoboClaw is not tuned, the motors will **NOT** move correctly.

1.  Connect RoboClaw to a Windows PC via USB.
2.  Open **BasicMicro Motion Studio**.
3.  Perform **Velocity Calibration** (Auto-Tune or Manual).
4.  Ensure that **Velocity P, I, D** and **QPPS** values are set and saved to EEPROM.
5.  *Test inside Motion Studio first to ensure motors can hold a target speed.*

---

## 🔌 Hardware Setup

### Common Pins (All Variants)

| Component | Pin / Value | Description |
| :--- | :--- | :--- |
| **RoboClaw RX** | GPIO 16 | Serial2 RX |
| **RoboClaw TX** | GPIO 17 | Serial2 TX |
| **NeoPixel** | GPIO 48 | Status LED |
| **Agent Baudrate** | 115200 | Serial (USB) Communication Speed |

### 2WD Configuration

| Setting | Value |
| :--- | :--- |
| RoboClaw Address | 0x80 |
| RoboClaw Baudrate | 38400 |

### 4WD Configuration

| Motor Driver | Address | Motors |
| :--- | :--- | :--- |
| Left (Driver 1) | 0x80 | M1: Left Front, M2: Left Rear |
| Right (Driver 2) | 0x81 | M1: Right Front, M2: Right Rear |
| RoboClaw Baudrate | 115200 | Both drivers |

---

## 📡 ROS 2 Interface

### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `cmd_vel_out` | `geometry_msgs/msg/Twist` | Target linear (x) and angular (z) velocity. |

### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `odom_esp` | `nav_msgs/msg/Odometry` | Robot odometry data (see variant-specific details below). |

### Odometry Details

#### 2WD Variant
- **Position**: X, Y, Theta calculated from encoder ticks
- **Velocity**: vx, vyaw from encoder speed readings
- **Covariance**: High confidence (0.0025) for vx and vyaw

#### 4WD Variant
- **Position**: Not calculated (set to 0)
- **Velocity**: vx, vyaw only (averaged from all 4 wheels)
- **Covariance**: 
  - vx: 0.1 (medium trust)
  - vyaw: 0.5 (low trust)
  - All pose values: 9999 (not measured)

### Frames
* **Frame ID:** `odom`
* **Child Frame ID:** `base_link`

---

## ⚙️ Key Features

1.  **Differential Drive Kinematics:**
    * Converts `cmd_vel` (m/s) to motor ticks/second (QPPS).
    * 4WD: Same velocity applied to front and rear wheels on each side.

2.  **Safety Timeout:**
    * If no `cmd_vel_out` message is received for **500ms**, the robot automatically stops to prevent runaways.

3.  **Auto-Reconnection:**
    * Implements a state machine to automatically detect agent disconnection and attempt reconnection without resetting the board.

4.  **Visual Feedback (NeoPixel):**
    * 🔴 **Blinking Red:** Waiting for micro-ROS Agent.
    * 🔵 **Blue:** Agent found, creating ROS entities (Nodes, Publishers).
    * 🟢 **Green:** Connected and Active.

5.  **Odometry Publishing:**
    * Published at **20Hz** (50ms interval).

---

## 🛠️ Configuration

You can adjust physical robot parameters at the top of each `.ino` file:

```cpp
#define WHEEL_DIA        0.192   // Wheel Diameter (Meters)
#define TRACK_WIDTH      0.495   // Distance between wheels (Meters)
#define TICKS_PER_REV    751.8   // Encoder ticks per revolution (223 RPM motor)
```

### 2WD Only
```cpp
#define LEFT_MOTOR_IS_M1 false   // Swap motor mapping if needed
#define RC_ADDRESS       0x80    // Single RoboClaw address
#define RC_BAUDRATE      38400   // RoboClaw baudrate
```

### 4WD Only
```cpp
#define RC_LEFT_ADDRESS   0x80   // Left motor driver address
#define RC_RIGHT_ADDRESS  0x81   // Right motor driver address
#define RC_BAUDRATE       115200 // RoboClaw baudrate
```

---

## 🚀 How to Run

### For 2WD/4WD (micro-ROS)

1.  **Start the micro-ROS Agent** on your Raspberry Pi (Host):
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
    ```

2.  Power on the ESP32 via USB.

3.  **Check the Status LED:**
    * Wait for the NeoPixel to turn **Green**. This indicates a successful connection to the agent.

4.  **Verify Data Stream:**
    ```bash
    ros2 topic list
    ros2 topic echo /odom_esp
    ```

5.  **Send velocity commands:**
    ```bash
    ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
    ```

### For Demo (WiFi)

1.  Update WiFi credentials in the code:
    ```cpp
    const char* ssid = "YourNetworkName";
    const char* password = "YourPassword";
    ```

2.  Upload and power on the ESP32.

3.  Check Serial Monitor for the IP address.

4.  Open a browser and navigate to the IP address.

5.  Use the web interface to control the rover:
    * ⬆️⬇️ Forward/Backward: 0.4 m/s for 1 meter
    * ⬅️➡️ Turn: 30°/s for 90 degrees

---

## 📦 Dependencies

To compile this code, you need the following libraries installed in Arduino IDE or PlatformIO:

* [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino) (2WD/4WD only)
* [RoboClaw](https://github.com/basicmicro/roboclaw_arduino_library) (BasicMicro)
* [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)

---

**Project:** Pathfinder AMR
