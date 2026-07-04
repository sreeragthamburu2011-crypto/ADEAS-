# ADEAS - ACCIDENT DETECTION & EMERGENCY ALERT SYSTEM FOR BLUETOOTH RC CAR

## Project Overview

This project is a Bluetooth controlled 4-wheel RC car using Arduino UNO R3. The car can move Forward, Backward, Left, Right using MIT App Inventor. 

**Additional feature: Accident Detection.** The car uses SW-18020P Vibration Sensor and MPU6050 Gyroscope. If the car hits any object, the system detects it, stops all motors immediately, turns on buzzer alarm, and locks the car. The car will not move until user presses RESET button in the app.

### Development Phases
- **Phase 1:** Movement control ✓
- **Phase 2:** Accident detection and lock system ✓

## Components Required

1. Arduino UNO R3 x 1
2. L298N Motor Driver Module x 1
3. 4x DC Geared Motors with Wheels x 4
4. HC-05 Bluetooth Module x 1
5. SW-18020P Vibration Sensor x 1
6. MPU6050 Gyroscope + Accelerometer Module x 1
7. 5V Buzzer x 1
8. LED 5mm x 1 with 220ohm Resistor
9. 7.4V Li-ion Battery for Motors x 1
10. 9V Battery or Power Bank for Arduino x 1
11. Robot Chassis, Jumper Wires

## Quick Start

1. **Install Arduino IDE** - https://www.arduino.cc/en/software
2. **Install MPU6050 Library** - Sketch → Include Library → Manage Libraries → Search "MPU6050"
3. **Upload Code** - Open `Arduino/ADEAS_Complete_Final_Code.ino` → Select Board → Upload
4. **Build Hardware** - Follow connections in `Docs/Hardware_Setup.md`
5. **Create MIT App** - Use blocks from `MIT_App_Inventor/ADEAS_Complete_Button_Configuration.txt`

## Commands

| Command | Function |
|---------|----------|
| F | Forward |
| B | Backward |
| L | Left |
| R | Right |
| S | Stop |
| r | Reset |
| W | LED ON |
| w | LED OFF |
| V | Buzzer ON |
| v | Buzzer OFF |

## Status

✅ Complete implementation ready for deployment
