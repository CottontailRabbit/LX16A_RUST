# LX16A_RUST
![image](https://github.com/CottontailRabbitt/LX16A_RUST/assets/99775161/539ca65e-1efe-43e1-b4ab-6ca92b3a63cd)

# ServoController

This Rust library provides a simple interface to control servo motors via serial communication. It supports various commands such as moving the servo to a specific position, reading its current position, and controlling other servo-related parameters like motor modes and LED states.

## Features

- **Move Servo**: Move the servo to a specific position within a specified time.
- **Motor Mode**: Switch between servo mode and motor mode with configurable speed.
- **Read Position**: Get the current position of the servo.
- **LED Control**: Turn the servo LED on or off.
- **Error Handling**: Handle different types of errors, such as timeouts or communication errors.

## Getting Started

To use this library, you'll need to install the following Rust crates:

- `serialport`: For serial communication.
- `log`: For logging and debugging.

### Prerequisites

Ensure you have Rust installed. You can get it [here](https://www.rust-lang.org/tools/install).

### Installation

To install and use the library, add the following dependencies to your `Cargo.toml` file:

```toml
[dependencies]
serialport = "3.3.0"
log = "0.4"
