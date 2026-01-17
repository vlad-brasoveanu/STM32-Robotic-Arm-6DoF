# 6-DOF Teach & Play Robotic Arm 🤖

[![Platform](https://img.shields.io/badge/Platform-STM32-blue)](https://www.st.com/)
[![Framework](https://img.shields.io/badge/Framework-FreeRTOS-orange)](https://www.freertos.org/)
[![Language](https://img.shields.io/badge/Language-C-green)](https://en.cppreference.com/w/c)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

A 6-Degree-of-Freedom robotic arm powered by **STM32 Nucleo-F401RE** and **FreeRTOS**.
This project features a "Teach & Play" mode, allowing users to manually record movements and replay them autonomously, similar to industrial "Cobots".

![Robot Demonstration](Images/robot_photo.jpg)


## 📖 Project Overview

This project aims to build an autonomous robotic manipulator capable of precise control and sequence repetition without requiring external PC software during operation. It leverages a Real-Time Operating System (FreeRTOS) to manage motor control, safety sensors, and user interface simultaneously.

### Key Features
* **Teach & Play:** Record up to 20 waypoints manually and replay them with smooth interpolation.
* **Dual Mode:** * *Manual Mode:* Fine control via physical buttons.
    * *Auto Mode:* Autonomous execution of saved paths (Retrace/Fast modes).
* **Safety First:**
    * Hardware **E-STOP** functionality using a relay cut-off.
    * **Current Monitoring** (Overcurrent protection) using ACS712 sensor.
    * Software angle limits to prevent mechanical collisions.
* **Standalone HMI:** 16x2 LCD Display and 8-button control panel.

## 🛠 Hardware Architecture

### Components
* **MCU:** STM32 Nucleo-F401RE (ARM Cortex-M4, 84MHz)
* **Actuators:** 6x PWM Digital Servos (MG996R for main axes, SG90 for gripper)
* **Sensors:** ACS712 Current Sensor (5A module)
* **Display:** LCD 1602 with I2C Backpack
* **Power:** External 5V/5A Power Supply with Relay Module for safety

### Pinout Configuration
| Component | Function | STM32 Pin | Note |
| :--- | :--- | :--- | :--- |
| **M1-M4** | Servo PWM | PC6 - PC9 | Timer 3 |
| **M5-Grip** | Servo PWM | PA8, PA9 | Timer 1 |
| **ACS712** | Current Sense | PA0 | ADC1 Channel 0 |
| **LCD** | I2C Display | PB8 (SCL), PB9 (SDA) | I2C1 |
| **Relay** | E-Stop Cutoff | PB12 | Digital Out |
| **Buttons** | User Inputs | Port C (PC0-PC3, PC10-PC11) | External 2000Ω Pull-Up |

> 📄 **Full Documentation:** For detailed schematics and technical specifications, please check the [Documentation PDF](Docs/Documentation.pdf).

## 💻 Software Architecture

The firmware is developed in **STM32CubeIDE** using HAL libraries and **FreeRTOS**. The system is divided into 4 independent tasks:

1.  **`StartTask_Monitor` (Critical Priority):** Runs every 20ms. Monitors E-STOP button and Current Sensor. Has the authority to cut power instantly.
2.  **`StartTask_Servo` (High Priority):** Generates precise PWM signals (50Hz) and handles angle-to-microsecond conversion.
3.  **`StartTask_Logic` (Normal Priority):** Handles the Finite State Machine (Manual/Record/Play), kinematics interpolation, and button debouncing.
4.  **`StartTask_HMI` (Low Priority):** Updates the LCD display via I2C asynchronously.

**Sync Mechanism:** A Mutex (`AnglesMutex`) is used to protect shared angle variables between the Logic and Servo tasks to prevent race conditions.

## 🚀 Getting Started

### Prerequisites
* STM32CubeIDE (v1.10 or newer)
* Drivers for ST-Link

### Installation
1.  Clone the repository:
    ```bash
    git clone https://github.com/vlad-brasoveanu/STM32-Robotic-Arm-6DoF
    ```
2.  Open **STM32CubeIDE** and select `File > Import > Existing Projects into Workspace`.
3.  Select the cloned folder.
4.  Open `RoboticArm.ioc` if you want to inspect the configuration (Optional).
5.  Build the project (Hammer icon).
6.  Connect your Nucleo board and click **Run/Debug**.

## 🎮 How to Use

1.  **Power On:** Ensure the external power supply is connected.
2.  **Manual Mode:** * Use `SELECT` to choose a motor (M1-M6).
    * Use `+` / `-` to move the arm.
    * Press `RECORD` to save a waypoint.
3.  **Auto Mode:**
    * Press `PLAY` to start the sequence.
    * Press `MODE` to toggle between *Retrace* (A->B->A) or *Fast Return* (A->B, jump to A).
4.  **Home:** Long press `MODE` (3s) to return to the safe position.
5.  **Emergency:** Press the `E-STOP` button to immediately kill power to servos.

## 🤝 Acknowledgments & Credits

* **3D Model:** The mechanical design of the robot arm is based on the work by FABRI_CREATOR found at https://cults3d.com/en/3d-model/gadget/brazo-robotico-con-arduino-step-files-robotic-arm-guardar-reproducir-export. 
    * *Specific changes made:* Added an RS-232 hole for the PWM signal cable, added personal logo, created the control box.
* **Libraries:** STM32 HAL Driver, FreeRTOS.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
