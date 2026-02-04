# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FTC (FIRST Tech Challenge) robotics project for Raven Robotics, DECODE 2025-2026 season. Built on FTC SDK v11.0 using Android Studio with Gradle.

## Build Commands

```bash
./gradlew build                    # Build all modules
./gradlew :TeamCode:build          # Build only TeamCode
./gradlew assembleDebug            # Create debug APK for deployment
./gradlew assembleRelease          # Create release APK
```

No automated test suite exists. Testing is done by deploying OpModes to the physical robot and running them via the Driver Station app.

## Architecture

### Module Structure

- **FtcRobotController/** - Core FTC SDK framework (library module, rarely modified)
- **TeamCode/** - All team-written robot code lives here

### TeamCode Organization

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── opmode/
│   ├── AutoMode/       # Autonomous routines (Red_Auto.java, Blue_Auto.java)
│   ├── TeleOpMode/     # Driver-controlled modes (MainRobotCode.java, SpeedGraph.java)
│   └── Concepts/       # Testing/calibration modes
└── util/               # Shared utilities (WaverlyGamepad.java)
```

### OpMode Patterns

OpModes are robot control programs that run on the Robot Controller. Two inheritance patterns:

- **Iterative** (`extends OpMode`): Implement `init()`, `start()`, `loop()`, `stop()`. Used for teleop.
- **Linear** (`extends LinearOpMode`): Implement `runOpMode()`. Used for autonomous.

Register with annotations:
```java
@TeleOp(name = "Display Name", group = "Robot")
@Autonomous(name = "Display Name")
@Disabled  // Hides from Driver Station
```

### Hardware Access Pattern

All hardware is accessed via `hardwareMap` in `init()`:
```java
DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "MotorName");
Servo servo = hardwareMap.get(Servo.class, "ServoName");
```

Hardware names must match the Robot Controller configuration file.

### Key Utilities

**WaverlyGamepad** (`util/WaverlyGamepad.java`): Wrapper for controller input that tracks button press events. Call `readButtons()` each loop iteration, then check:
- `button` - currently held
- `buttonPressed` - just pressed this frame (edge detection)

### Robot Hardware Configuration

Drive: 4 mecanum wheels (FrontLeft, FrontRight, BackLeft, BackRight)
Scoring: RightShooter, LeftShooter motors; Intake, Belt motors; ballLift, ballRamp servos
Sensors: IMU, Limelight3A camera, AprilTag detection
Feedback: Rev Blinkin LED driver

### Vision Integration

- **AprilTagProcessor**: Tag detection for localization
- **Limelight3A**: Distance/angle calculation to game elements using `(TAG_HEIGHT - CAMERA_HEIGHT) / tan(angle)`

## Build Configuration

- Java 8, Android SDK 30 (min 24, target 28)
- NDK 21.3.6528147, ABIs: armeabi-v7a, arm64-v8a
- Debug keystore: `libs/ftc.debug.keystore`