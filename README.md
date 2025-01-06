# Line Following Robot with Remote Control

This project implements a line-following robot using the TMS320F28379D microcontroller, with wireless remote control functionality via an ESP32 module. The robot uses PID control to follow a line based on sensor feedback. Its speed and other parameters can be remotely adjusted. The system also sends RPM and power consumption data back to the remote controller.

The line-following robot uses encoders to measure wheel speed and reflective sensors to detect the line. This project is ideal for learning about embedded systems, motor control, and communication between devices.

![line-following-robot-with-remote-control](https://github.com/user-attachments/assets/b3240738-2230-4c65-9771-0f225a6ff7f0)


## 🚀 Features

- **Line Following**: The robot autonomously follows a line using reflective sensors.
- **PID Control**: The robot uses a PID control algorithm to adjust motor speeds based on line detection.
- **Remote Control**: Controls the robot's speed and on/off state via an ESP32-based remote.
- **Speed and Power Monitoring**: Monitors and sends RPM and power consumption data back to the remote.
- **Real-Time Adjustment**: PID constants and other settings can be adjusted remotely.
- **Error Handling**: Avoids errors such as divide-by-zero in speed calculations.
- **Power Efficiency**: Measures and sends real-time power consumption data.

## 🖱️Hardware Requirements

- **TMS320F28379D** (or compatible device)
- **ESP32** x 2 (for remote control and communication)
- **Reflective Sensors** x 7 (for line detection)
- **Encoders** x 2 (for wheel speed measurement)
- **Motors and wheels** x 2 (for movement)
- **Power Supply** x 3 (to power each microcontroller)

## 💻Software Requirements

- **TI Code Composer Studio**: For development and programming the TMS320F28379D.
- **ESP32 Firmware**: For communication and remote control.
- **Libraries**:
    - TI BIOS (for real-time OS management)
    - Standard TI libraries for GPIO, PWM, ADC, and UART.
    - TI F2837xD_GlobalVariableDefs.c

### **Code Breakdown**

Below is a brief description of each thread:

- **encoder_1_hwi()** and **encoder_2_hwi()**: These functions update the last and current time variables for the respective encoders.
- **calculate_rpm1_swi()** and **calculate_rpm2_swi()**: These functions calculate the RPM values by measuring the time between two encoder ticks.
- **msTimerFxn()**: This function triggers posts for control_loop_swi() and updatePower(), as well as updates the isrFlag.
- **control_loop_swi()**: This function reads the reflective array sensors, calculates the error of reflective array sensors, computes the PID value based of error and Kp, Kd, Ki and updates the PWM motor values according to PID.
- **myIdleFxn()**: The heartbeat red LED is activated when isrFlag is set to true.
- **sci_rx_hwi()**: This interrupt service routine retrieves data from the ESP32 and posts it to rxData_swi.
- **rxData_swi()**: This function processes the data received from the ESP32.
- **txData_swi()**: This function measures the two ADCs, calculates power consumption, and sends both the power and RPM1 values.

## 🔧 **Getting Started**

### 1. **Clone the Repository**

To get started, follow these simple steps:

- **Step 1**: Install Git (if not already installed) from [here](https://git-scm.com/downloads).
- **Step 2**: Clone the repository by running the following command on your terminal or command prompt:
    
    ```bash
    bash    git clone https://github.com/jasmine-portfolio/Line-Following-Robot-with-Remote-Control.git
    ```
    
- **Step 3**: Set up your TMS320F28379D development environment and upload the code.
- **Step 4**: Set up the ESP32 and configure communication between the robot and the remote.

### 2. **Alternative: Download as ZIP**

1. **Step 1**: Download the repository as a ZIP file.
2. **Step 2**: Extract the folder to your local machine.
3. **Step 3**: Connect the components to the TMS320F28379D board as per the hardware setup.
4. **Step 4**: Load the code onto the microcontroller.

## 🏗️ How It Works

The system operates through a combination of peripherals, periodic tasks, interrupts, and a control loop. Here's a breakdown of its operation:

### 1. **System Initialization**

- **Peripherals Setup**: Initializes various system peripherals, including:
    - **PWM** for motor control.
    - **ADC** for measuring power consumption.
    - **UART** for serial communication between the robot and external devices (e.g., ESP32).

### 2. **Periodic Tasks (Timer-Triggered)**

- **PID Control Loop (every 1ms - `control_loop_swi`)**:
    - The reflective array sensors are polled to determine which sensor is triggered.
    - Based on this sensor data, the error (deviation from the path) is calculated.
    - The **PID algorithm** adjusts the PWM motor speeds to correct the robot’s movement and maintain its desired trajectory.
- **Data Transmission (every 1s - `txData_swi`)**:
    - Every second, the robot calculates its **power consumption** and sends both **RPM** and **power data** to an external controller (like the ESP32) for monitoring.

### 3. **Interrupts**

- **Encoder Speed Calculation (via GPIO XINT - `encoder_hwi`)**:
    - The system tracks the motor speed by measuring encoder pulses.
    - The time difference between successive encoder pulses is used to calculate the **RPM** (Revolutions Per Minute) of each motor.
- **Serial Communication Reception (via GPIO RX - `sci_rx_hwi`)**:
    - Commands are received from an external controller (e.g., ESP32) via UART.
    - These commands control various aspects of the robot, such as its state, speed, and speaker settings.

### 4. **Idle State (via `myIdleFxn`)**

- **Heartbeat LED**: The system has a heartbeat indicator—a red LED that flashes to show the system is running.

![image](https://github.com/user-attachments/assets/7dd8c828-a0ca-4447-bc96-51c1fa760bfd)


## 🕹️ Control Modes

- **PID Control**: The robot autonomously adjusts motor speed to follow the line. You can adjust the PID constants via the remote control.
- **Manual Control**: The robot's state (on/off) can be switched remotely, and its speed can be set using the ESP32.

## 🛠️ Troubleshooting

- **If the robot is not following the line properly**:
    - Check the alignment and calibration of the reflective sensors.
    - Adjust the PID constants for better line-following performance.
- **If the remote control is not working**:
    - Ensure that the ESP32 is properly paired and communicating with the TMS320F28379D.
    - Verify the UART connection between the ESP32 and the TMS320F28379D is set up correctly.
    - Make sure the watchdog timer is disabled

## 📄 **License**

This project is licensed under the MIT License. Feel free to fork, modify, and use it for your own portfolio.
