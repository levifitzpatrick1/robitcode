## Table of Contents

-   [[#Introduction]]
-   [[#Commands]]
    -   [[#Drive Commands]]
    -   [[#Vision Commands]]
-   [[#Constants]]
-   [[#Main Files]]
-   [[#Subsystems]]
-  [[#Additional Information]]

## Introduction

This repository contains the codebase for a robot designed for FRC competitions. The code is primarily written in Java and utilizes the WPILib framework. It is organized into various packages, classes, and methods to control different aspects of the robot's functionality.

## Commands

Commands are actions that the robot can perform. They are organized into different categories for easier understanding.

### Drive Commands

#### [ResetHeadingCmd.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Commands/DriveCommands/ResetHeadingCmd.java)

-   **Purpose**: Resets the robot's heading to a default value.
-   **Usage**: Typically used when the robot needs to align itself to a specific direction.

#### [ResetOdometry.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Commands/DriveCommands/ResetOdometry.java)

-   **Purpose**: Resets the robot's odometry, which is the data representing its position and orientation.
-   **Usage**: Useful for recalibrating the robot's understanding of its location on the field.

#### [SwerveJoystickCmd.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Commands/DriveCommands/SwerveJoystickCmd.java)

-   **Purpose**: Allows the robot to move using a joystick in a swerve drive configuration.
-   **Usage**: Used for manual control of the robot during matches.

### Vision Commands

#### [TrackTargetIDPosCmd.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Commands/VisionCommands/TrackTargetIDPosCmd.java)

-   **Purpose**: Tracks a target based on its position ID.
-   **Usage**: Useful for aligning the robot with game elements or objectives.

#### [TrackTargetIDRotCmd.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Commands/VisionCommands/TrackTargetIDRotCmd.java)

-   **Purpose**: Tracks a target based on its rotation ID.
-   **Usage**: Helps the robot to rotate and align itself with a target.

## Constants

Constants are values that are used throughout the code but do not change. They are organized into different files for better manageability.

### [Constants.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Constants/Constants.java)

-   **Purpose**: Contains miscellaneous constants used in various parts of the code.

### [FieldConstants.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Constants/FieldConstants.java)

-   **Purpose**: Contains constants related to the dimensions and features of the competition field.

### [ModuleConstants.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Constants/ModuleConstants.java)

-   **Purpose**: Contains constants specific to the swerve drive modules, such as speed limits and motor IDs.

## Main Files

### [Main.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Main.java)

-   **Purpose**: Acts as the entry point for the robot code.
-   **Usage**: Initializes the robot program.

### [Robot.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Robot.java)

-   **Purpose**: Contains the main robot class with methods for initialization, periodic updates, and state changes.
-   **Usage**: Central hub for managing robot behavior.

### [RobotContainer.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/RobotContainer.java)

-   **Purpose**: Sets up the robot's command structure and subsystems.
-   **Usage**: Used for initializing and linking commands to subsystems.

## Subsystems

Subsystems are components of the robot that have specific functionalities. They are controlled by commands.

### [Drivetrain.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Subsystems/Drivetrain.java)

-   **Purpose**: Manages the robot's drivetrain, including motors and sensors.
-   **Usage**: Used in various commands to control the robot's movement.
- - [**More Info**](obsidian://open?vault=Vaulty%20Boi&file=Robotics%2FDocumentation%2FFile%20Specific%2FDrivetrain%20Class%20Documentation)

### [PhotonCameraWrapper.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Subsystems/PhotonCameraWrapper.java)

-   **Purpose**: Provides a wrapper around the Photon Camera for vision tracking.
-   **Usage**: Used in vision commands to identify and track targets.

### [SwerveModule.java](https://github.com/levifitzpatrick1/robitcode/blob/main/plz/src/main/java/frc/robot/Subsystems/SwerveModule.java)

-   **Purpose**: Represents a single swerve drive module.
-   **Usage**: Used in the drivetrain subsystem to control individual modules.
- [**More Info**](obsidian://open?vault=Vaulty%20Boi&file=Robotics%2FDocumentation%2FFile%20Specific%2FSwerve%20Module%20Class%20Documentation)

## Additional Information

For more details, please refer to the [README.md](https://github.com/levifitzpatrick1/robitcode/blob/main/README.md) file in the repository.