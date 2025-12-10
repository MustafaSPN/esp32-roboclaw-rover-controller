# 🤖 ESP32 RoboClaw Rover Controller

This firmware is developed for the **Pathfinder** autonomous mobile robot. It serves as the low-level hardware interface, bridging the **ROS 2 Humble** network with the **RoboClaw** motor controller via **micro-ROS**.

It handles differential drive kinematics, odometry calculation (Forward Kinematics), and safety mechanisms.

![Micro-ROS](https://img.shields.io/badge/Micro--ROS-Humble-blue) ![ESP32](https://img.shields.io/badge/Board-ESP32-green) ![License](https://img.shields.io/badge/License-MIT-yellow)

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

## 🔌 Hardware Setup

| Component | Pin / Value | Description |
| :--- | :--- | :--- |
| **RoboClaw RX** | GPIO 16 | Serial2 RX |
| **RoboClaw TX** | GPIO 17 | Serial2 TX |
| **NeoPixel** | GPIO 48 | Status LED |
| **Agent Baudrate** | **115200** | Serial (USB) Communication Speed |
| **RC Baudrate** | 38400 | RoboClaw Communication Speed |
| **Address** | 0x80 | RoboClaw Packet Serial Address |

## 📡 ROS 2 Interface

### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Target linear (x) and angular (z) velocity. |

### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | Calculated robot position (X, Y, Theta) and velocity based on encoder feedback. |

### Frames
* **Frame ID:** `odom`
* **Child Frame ID:** `base_link`

## ⚙️ Key Features

1.  **Differential Drive Kinematics:**
    * Converts `cmd_vel` (m/s) to motor ticks/second (QPPS).
    * Calculates odometry from encoder ticks.
2.  **Safety Timeout:**
    * If no `/cmd_vel` message is received for **500ms**, the robot automatically stops to prevent runaways.
3.  **Auto-Reconnection:**
    * Implements a state machine to automatically detect agent disconnection and attempt reconnection without resetting the board.
4.  **Visual Feedback (NeoPixel):**
    * 🔴 **Blinking Red:** Waiting for micro-ROS Agent.
    * 🔵 **Blue:** Agent found, creating ROS entities (Nodes, Publishers).
    * 🟢 **Green:** Connected and Active.

## 🛠️ Configuration

You can adjust physical robot parameters at the top of the `.ino` file:

```cpp
#define WHEEL_DIA        0.192   // Wheel Diameter (Meters)
#define TRACK_WIDTH      0.55    // Distance between wheels (Meters)
#define TICKS_PER_REV    145.6   // Encoder ticks per revolution
#define LEFT_MOTOR_IS_M1 false   // Swap motor mapping if needed
```

## 🚀 How to Run

1.  **Start the micro-ROS Agent** on your Raspberry Pi (Host) with the correct baudrate:
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
    ```
2.  Power on the ESP32 via USB.
3.  **Check the Status LED:**
    * Wait for the NeoPixel to turn **Green**. This indicates a successful connection to the agent.
4.  **Verify Data Stream:**
    Open a new terminal and check if the odometry data is being published:
    ```bash
    ros2 topic list
    ros2 topic echo /odom
    ```

## 📦 Dependencies

To compile this code, you need the following libraries installed in Arduino IDE or PlatformIO:

* [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino)
* [RoboClaw](https://github.com/basicmicro/roboclaw_arduino_library) (BasicMicro)
* [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)

---
**Project:** Pathfinder AMR
