# Real-Time Motion Detection Alarm System

This repository contains the code and configurations for a real-time motion detection alarm system, developed as part of our Real-Time Systems Design class.

## Project Overview

The Real-Time Motion Detection Alarm System is built using an STM32 microcontroller (STM32F446RE). The system is designed to detect motion and respond with visual and auditory alarms. It incorporates FreeRTOS to handle multiple real-time tasks, including keypad input, sensor monitoring, and light controls.

### Key Features
- **Real-time motion detection**: Uses a motion sensor to detect movement in real time.
- **Keypad input**: A 4x4 keypad allows users to input a security code to activate or deactivate the system.
- **Alarms**: The system triggers alarms (both lights and sounds) when unauthorized motion is detected.
- **OLED display**: Displays the current system status (armed/disarmed) and keypad interactions.
- **Multitasking with FreeRTOS**: Efficient task management using the FreeRTOS real-time operating system.

## Hardware Components
- STM32F446RE Microcontroller
- 4x4 Keypad for input
- Motion Sensor (PIR)
- OLED Display (SSD1306)
- LEDs for visual alarms
- Buzzer for auditory alarms
- I2C and UART interfaces for communication

## Software Components
- **FreeRTOS**: Used to manage tasks and ensure real-time performance.
- **STM32 HAL**: STM32 Hardware Abstraction Layer libraries to interface with the hardware components.
- **CMSIS-OS**: A CMSIS-compliant RTOS API for task management.
- **Custom Drivers**: Includes custom drivers for the keypad and OLED display.

### Main Tasks
1. **ReadInputTask**: Handles user input via the keypad to arm/disarm the system.
2. **ReadSensorTask**: Continuously monitors the motion sensor to detect any unauthorized movement.
3. **RedLight Task**: Manages the red LED, which signals an alarm state.
4. **GreenLight Task**: Controls the green LED, indicating system readiness.
5. **WriteInputTask**: Updates the OLED display based on user input and system state.

## Code Structure

- `Core/Inc`: Header files for system configuration and driver interfaces.
- `Core/Src`: Source files implementing the main logic of the system, including motion detection, task scheduling, and keypad interaction.
  - `main.c`: Contains the main program logic, including FreeRTOS task creation and system initialization.
  - `Keypad4X4.c`: Manages input from the 4x4 keypad.
  - `ssd1306.c`: OLED display control code.
- `Drivers`: Contains STM32 peripheral drivers.

## Getting Started

### Prerequisites
- STM32CubeIDE or compatible IDE for development
- STM32CubeMX for generating initialization code
- STM32F446RE development board
- FreeRTOS library configured for STM32

### Building the Project
1. Clone the repository to your local machine.
2. Open the project in STM32CubeIDE.
3. Build the project to generate the binary files.
4. Flash the binary to the STM32F446RE board using the ST-Link programmer.

### Running the System
1. Connect the motion sensor, keypad, LEDs, and buzzer to the STM32 board as per the circuit diagram.
2. Power the system and observe the OLED display for the system status.
3. Use the keypad to arm/disarm the system with a predefined security code.
4. If motion is detected while the system is armed, the alarm will trigger.

## Future Enhancements
- Add password change functionality via keypad.
- Implement remote control capabilities via UART or Bluetooth.
- Improve the user interface on the OLED display.

## License
This project is licensed under the terms that can be found in the LICENSE file provided with the code.

## Acknowledgments
- STMicroelectronics for providing the STM32 development tools.
- FreeRTOS for enabling real-time task management.
