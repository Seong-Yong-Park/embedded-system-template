# Software Requirements Specification (SRS)

## 1. Overview
### 1.1 Purpose
This document specifies the software requirements for the [Project Name] embedded system. It serves as a contract between stakeholders and the development team, defining what the system shall do without specifying how it will be implemented.

**Example:**
- Define functional and non-functional requirements for the motor control system
- Establish requirements baseline for design and testing
- Provide traceability from requirements to implementation

### 1.2 Scope
This SRS covers the embedded software for [Project Name], including:
- **In Scope:**
  - Firmware for MCU-based control system
  - Communication protocols (CAN, UART, I2C)
  - Sensor data acquisition and processing
  - Actuator control algorithms
  
- **Out of Scope:**
  - PC-side configuration tools (covered in separate document)
  - Hardware design (covered in hardware specification)
  - Manufacturing and production processes

### 1.3 Definitions and Acronyms
| Term | Definition |
|------|------------|
| MCU | Microcontroller Unit |
| HAL | Hardware Abstraction Layer |
| BSP | Board Support Package |
| RTOS | Real-Time Operating System |
| HIL | Hardware-In-the-Loop |
| CAN | Controller Area Network |
| PWM | Pulse Width Modulation |
| ADC | Analog-to-Digital Converter |

### 1.4 References
- [Project Vision](00_project-vision.md)
- [System Architecture Document](20_SAD.md)
- [Interface Specification](40_InterfaceSpec.md)
- Hardware datasheet: [MCU Model] Datasheet vX.X
- Standard: ISO 26262 (if applicable)

## 2. Overall Description
### 2.1 Product Perspective
The [Project Name] is a component of a larger system that [describe system context].

**Example:**
- The motor controller is part of an automated manufacturing line
- It interfaces with a central PLC via CAN bus
- It receives commands from the HMI system
- It controls multiple servo motors and sensors

### 2.2 Product Functions
The system shall provide the following major functions:

**Example:**
- **FR-001**: Motor speed control with PID algorithm
- **FR-002**: Temperature monitoring and overheat protection
- **FR-003**: CAN bus communication for remote control
- **FR-004**: Fault detection and diagnostic reporting
- **FR-005**: Configuration storage in non-volatile memory

### 2.3 User Characteristics
**Example:**
- **Primary Users**: Factory operators with basic technical knowledge
- **Secondary Users**: Maintenance technicians with embedded systems experience
- **System Integrators**: Engineers integrating the controller into larger systems
- **No specialized training required** for basic operation

### 2.4 Constraints
**Example:**
- **Hardware Constraints:**
  - MCU: STM32F4 series (limited to 168 MHz, 1MB Flash, 192KB RAM)
  - Operating temperature: -40°C to +85°C
  - Power supply: 12V ±10%
  
- **Software Constraints:**
  - Must use FreeRTOS (company standard)
  - Code must comply with MISRA C:2012 guidelines
  - Maximum boot time: 2 seconds
  
- **Regulatory Constraints:**
  - Must comply with CE marking requirements
  - EMC standards: EN 55032, EN 55035

## 3. Functional Requirements
### 3.1 Functional Requirement 1
**FR-001: Motor Speed Control**

**Description:** The system shall control motor speed using a PID control algorithm.

**Inputs:**
- Target speed command (0-3000 RPM)
- Current speed feedback from encoder

**Outputs:**
- PWM signal to motor driver (0-100% duty cycle)

**Processing:**
- PID controller with configurable Kp, Ki, Kd parameters
- Update rate: 1 kHz
- Anti-windup protection

**Example:**
```
REQ-001.1: The system shall accept speed commands in the range 0-3000 RPM
REQ-001.2: The system shall achieve target speed within ±5 RPM within 2 seconds
REQ-001.3: PID parameters shall be configurable via CAN interface
```

### 3.2 Functional Requirement 2
**FR-002: Temperature Monitoring**

**Description:** The system shall monitor motor temperature and implement protection.

**Inputs:**
- Temperature sensor reading (analog or digital)

**Outputs:**
- Temperature status (normal/warning/critical)
- Motor shutdown command if critical threshold exceeded

**Processing:**
- Read temperature every 100ms
- Compare against configurable thresholds
- Implement thermal derating if warning threshold reached

**Example:**
```
REQ-002.1: System shall read temperature sensor every 100ms
REQ-002.2: Warning threshold: 70°C (configurable)
REQ-002.3: Critical threshold: 85°C (configurable)
REQ-002.4: System shall reduce motor power by 50% at warning threshold
REQ-002.5: System shall stop motor at critical threshold
```

<!-- Additional functional requirements -->
### 3.3 Functional Requirement 3
**FR-003: CAN Communication**

**Description:** The system shall communicate with external devices via CAN bus.

**Example:**
```
REQ-003.1: System shall support CAN 2.0B protocol
REQ-003.2: Baud rate: 500 kbps (configurable: 125, 250, 500, 1000 kbps)
REQ-003.3: System shall respond to command messages within 10ms
REQ-003.4: System shall transmit status messages at 100ms intervals
```

## 4. Non-Functional Requirements
### 4.1 Performance Requirements
**Example:**
- **NFR-001**: Control loop execution time: ≤ 1ms (1 kHz update rate)
- **NFR-002**: CAN message processing latency: ≤ 10ms
- **NFR-003**: Boot time: ≤ 2 seconds from power-on to ready state
- **NFR-004**: Memory usage: ≤ 80% of available RAM
- **NFR-005**: CPU utilization: ≤ 70% average, ≤ 95% peak

### 4.2 Safety Requirements
**Example:**
- **NFR-006**: System shall implement watchdog timer (timeout: 500ms)
- **NFR-007**: Critical faults shall trigger safe shutdown within 100ms
- **NFR-008**: All safety-critical functions shall be executed in high-priority tasks
- **NFR-009**: System shall detect and report sensor failures
- **NFR-010**: Emergency stop input shall override all other commands

### 4.3 Reliability Requirements
**Example:**
- **NFR-011**: Mean Time Between Failures (MTBF): ≥ 10,000 hours
- **NFR-012**: System shall recover from non-critical faults without manual intervention
- **NFR-013**: Data integrity: CRC checksum for all stored configuration data
- **NFR-014**: System shall maintain operation during brown-out conditions (9V-15V)
- **NFR-015**: Flash memory endurance: ≥ 10,000 write cycles

### 4.4 Maintainability Requirements
**Example:**
- **NFR-016**: Code shall follow MISRA C:2012 guidelines
- **NFR-017**: All functions shall have Doxygen-style documentation
- **NFR-018**: Code coverage for unit tests: ≥ 80%
- **NFR-019**: System shall provide diagnostic information via CAN (fault codes, status)
- **NFR-020**: Firmware update capability via CAN or UART bootloader

## 5. Interface Requirements
### 5.1 User Interface
**Example:**
- **UI-001**: Status LED indicators (Power, Run, Fault, Communication)
- **UI-002**: 4-digit 7-segment display showing current speed (optional)
- **UI-003**: Push button for local start/stop control
- **UI-004**: Rotary encoder for speed adjustment (optional)

### 5.2 Hardware Interface
**Example:**
- **HW-001**: Motor driver interface: PWM signal (0-3.3V, 20kHz)
- **HW-002**: Encoder interface: Quadrature encoder (A/B channels, 5V tolerant)
- **HW-003**: Temperature sensor: Analog input (0-3.3V) or I2C digital sensor
- **HW-004**: Emergency stop: Digital input (active low, 24V)
- **HW-005**: Power supply monitoring: ADC input (0-3.3V, scaled from 0-15V)

### 5.3 Software Interface
**Example:**
- **SW-001**: Configuration API: Functions to read/write configuration parameters
- **SW-002**: Diagnostic API: Functions to retrieve fault codes and status
- **SW-003**: Control API: Functions to start/stop motor, set target speed
- **SW-004**: Logging API: Functions to log events to non-volatile storage

### 5.4 Communication Interface
**Example:**
- **COM-001**: CAN bus: 500 kbps, 29-bit extended IDs
  - Command messages: 0x18XX XXXX (received by controller)
  - Status messages: 0x19XX XXXX (transmitted by controller)
- **COM-002**: UART: 115200 baud, 8N1, for debug and firmware update
- **COM-003**: I2C: 400 kHz, for sensor communication
- **COM-004**: SPI: 1 MHz, for external memory (if applicable)
