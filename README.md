# GPS master clock
This project is intended to replace old (radio-based) master clocks. This is done with cheap, off the shelf components.
## Features
- Accurate timekeeping via GPS
- Drive two lines of electromechanical slave clocks independently
- Recovery in case of power outage
- Short circuit detection/protection
## Hardware required
- ESP32 WROOM board
- NEO6M GPS module
- L298 driver board
- Generic 16x2 LCD, I2C interface
- Some additional resistors, a Schottky diode and a 4700µF Cap
