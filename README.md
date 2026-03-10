# STM32 Real-Time Sensor Monitoring System

An embedded real-time sensor monitoring system built on **STM32** using **FreeRTOS**, capable of collecting accelerometer data, performing real-time step detection, logging records to an SD card, and providing an interactive LCD interface.

![demo](https://github.com/user-attachments/assets/9ae4cbe9-e9a3-4a62-9b0d-85cabae83635)

## Key Features

### User Interface

The device uses an **LCD display** to provide system feedback. Allows users to:

* Start/Stop recording sessions
* View real-time step counts
* Browse historical records

Example real-time display:

```
Time: 12 s
Rec Steps: 15
X:0.03   Y:-0.12
Z:0.98
```

### Real-Time Sensor Processing

The system reads acceleration data from the **ADXL accelerometer** and computes the acceleration magnitude:

```
acc_mag = sqrt(ax * ax + ay * ay + az * az);
acc_mag = acc_mag - 1.0f;
```

A **threshold crossing method** with debounce control is used for step detection.

```
if (prev_acc_mag < STEP_THRESHOLD && acc_mag >= STEP_THRESHOLD)
```

This ensures stable step counting by filtering noise and avoiding multiple triggers.

### Data Logging

Session records are stored in **CSV format** on the SD card.

Example:

```
0,120,45000
1,95,38000
2,150,60000
```

Each record contains:

* Session index
* Number of steps
* Duration

### Historical Record Browsing

Historical records can be loaded from the SD card and displayed on the LCD interface, allowing users to review past activity sessions.

Example display:

```
Rec: (1/3)
Steps: 95
Time: 38 s
```

## System Overview

![Achitecture](https://github.com/user-attachments/assets/dc607146-f895-4321-a066-1e8e229959ef)

### FreeRTOS Multi-Task Design

* **ButtonTask**: User interface state machine and button event handling
* **PedometerTask**: Sensor data acquisition and step detection
* **RecordTask**: Persistent storage management and SD card operations

### Inter-task Communication

* **Task Notifications**: For signaling state changes and events between tasks.
* **Queues**: For passing step count and session data between tasks.
* **Mutex**: To protect shared resources like the SD card filesystem.

### Peripheral Interfaces

| Peripheral     | Purpose                |
| -------------- | ---------------------- |
| GPIO Interrupt | Button event detection |
| I2C            | LCD Display, ADXL      |
| SPI            | SD Card communication  |
| UART           | Debug logging          |

### System States (FSM)

![FSM](https://github.com/user-attachments/assets/96fa90ad-95b3-416c-8664-3985573e0ea0)

## Hardware Components

* STM32 Microcontroller
* ADXL Accelerometer Sensor
* I2C LCD Display
* SD Card

## Development Environment

* STM32CubeIDE
* FreeRTOS
* FatFS
* HAL Drivers
