# DoubleScara Controller

This is the firmware for an autonomous puzzle-solving robot. This project was part of a module of my study program, developed by an interdisciplinary team of mechanical, electrical, and software engineers.

The firmware runs on a ![TinyK22](https://github.com/ErichStyger/mcuoneclipse/tree/master/tinyK22) and is responsible for receiving high-level commands (e.g., "Go to coordinate XY", "Grab part") from a Raspberry Pi host and translating them into precise robotic movements.

## Tech Stack & Tools

* **Language:** C
* **IDE:** MCUXpresso (Eclipse-based IDE by NXP)
* **Configuration:** NXP Config-Tool (for Pin-Muxing, Clock Tree, and peripheral initialization)
* **Operating System:** FreeRTOS

---

## Firmware Architecture

The firmware is designed using a modular, four-layer architecture to cleanly separate hardware dependencies from high-level application logic.

### 1. Application Layer

Coordinates the overall execution flow and system initialization.

* **Init Module:** Manages the system startup, motor configurations, and physical homing routines (using optical end-switches for the arms and hard-stops for the gripper/platform).
* **Statemachine Module:** Implements a robust 6-state control loop (`IDLE`, `EXECUTE`, `DONE`, `ERROR`, `ESTOP`, `RESET`) to safely process incoming commands.

### 2. Abstraction Layer

Translates hardware drivers into logical interfaces for the application.

* **SCARA Kinematics:** Converts XY-coordinates into motor angles using inverse kinematics. Features a custom momentum-based swing routine to cleanly pass through mechanical singularities without physical jerking.
* **Motor & Grabber Modules:** Abstracts step generation and TMC2209 configurations into simple commands (e.g., relative/absolute angles) and handles 3-step grabbing sequences.
* **E-Stop Module:** Monitors a global Emergency Stop flag via an external button or CLI command, instantly halting motor execution and triggering an MCU reset upon resolution.

### 3. Driver Layer

Handles direct communication with the physical hardware components.

* **Step Generation Module:** Generates precise stepper signals using the MCU's Flexible Timer Module (FTM). It computes trapezoidal or triangular motion profiles using a Taylor-series approximation algorithm (David Austin method) to offload complex square-root math into fast, pure **integer mathematics**. Supports synchronous multi-motor movements.
* **TMC2209 Module:** Configures stepper driver current limits, microstepping, and freewheeling over a single-wire UART interface.
* **Peripherals:** Drivers for the **PCA9555** I2C GPIO expander, an **SSD1306** OLED display (adapted from Erich Styger's McuLib), and status LEDs.

### 4. Infrastructure Layer

Provides vital system services, diagnostic middleware, and inter-task communication.

* **FreeRTOS Real-Time Execution:** Operates with 8 concurrent, dedicated tasks (`Log`, `CLI`, `Step`, `TMC2209`, `Motor`, `Motor Command`, `Scara Kinematics`, `Statemachine`).
* **Command Line Interface (CLI):** Utilizes *FreeRTOS+CLI* to handle human-machine communication. Dynamically switchable between the virtual COM port and **SEGGER RTT** via macros.
* **Logging System:** Thread-safe, queue-driven logging system that writes binary data to an SD card using **FatFS** middleware, with dynamic log-level filtering over the CLI.
* **Task Helpers:** A custom, asynchronous command-handle communication architecture utilizing FreeRTOS Direct-to-Task notifications, enabling non-blocking multi-command execution across tasks.

---

## Host Communication & Protocol

* **Interface:** UART2 connection to the Raspberry Pi with non-blocking `UART_RTOS_Receive` timeouts.
* **Data Format:** Comma-Separated Values (CSV) parsed via `strtok` into structured command packets (`CommandPackage`) containing the instruction, $X/Y/\text{Rotation}$ coordinates, and a Transaction ID (`transID`).
* **Handshake & Error Resilience:** Employs a symmetric `OK,<transID>` / `ERROR,<transID>` response scheme paired with a pre-flight startup handshake to guarantee synchronization and support automated retry mechanisms on the host side.
