## Overview

The `Drivetrain` class extends `SubsystemBase` and is responsible for controlling the swerve modules and handling the robot's position on the field. It also integrates with vision systems for target tracking.

## Import Statements

Various libraries and modules are imported to provide functionalities like PID control, kinematics, and data logging.

## Class Variables

- **Swerve Modules**: Four swerve modules (`frontLeftModule`, `frontRightModule`, `backLeftModule`, `backRightModule`) are defined.
- **Gyroscopes**: A Pigeon2 and an AHRS gyroscope are used for orientation.
- **PID Controllers**: PID controllers for angular and translational movements are defined.
- **PhotonCameraWrapper**: A wrapper for the Photon camera is used for vision.
- **SwerveDrivePoseEstimator**: Estimates the robot's pose on the field.

## Constructor: `Drivetrain()`

Initializes the swerve modules, gyroscopes, and vision systems. It also zeros the heading.

## Periodic Method: `periodic()`

Updates SmartDashboard with various sensor readings and updates the PID controller values.

---

### Commands and Methods

#### `getLowestSpeed()`

- **Description**: Normalizes wheel speeds.
- **Returns**: Lowest speed among all modules.

#### `getChassisSpeeds()`

- **Description**: Gets the current state of the drivetrain.
- **Returns**: Chassis speeds based on the states of the swerve modules.

#### `getModulePositions()`

- **Description**: Gets the current positions of the swerve modules.
- **Returns**: An array of `SwerveModulePosition`.

#### `getHeading()`

- **Description**: Gets the current robot heading from the gyro.
- **Returns**: Heading in degrees.

#### `getRotation2d()`

- **Description**: Gets the current robot rotation from the gyro.
- **Returns**: A `Rotation2d` object.

#### `resetOdometry(Pose2d pose)`

- **Description**: Resets the current odometry to the given pose.

#### `updateOdometry()`

- **Description**: Updates the robot's position using encoders, gyro, and vision.

#### `stopModules()`

- **Description**: Stops all swerve modules.

#### `zeroHeading()`

- **Description**: Resets the gyro to 0.

#### `setModuleStates(SwerveModuleState[] desiredStates)`

- **Description**: Sets the desired states for the swerve modules.

#### `trackTargetIDRotation(Integer targetID)`

- **Description**: Tracks the rotation of a target with a given ID.
- **Returns**: Rotation speed to track the target.
- [**More Info**](https://github.com/levifitzpatrick1/robitcode/blob/main/Documentation/File%20Specific/Complex%20Functions/trackTargetIDRotation(Integer%20targetID).md)

#### `trackTargetIDPosition(Integer targetID)`

- **Description**: Follows the target while maintaining an X distance of 1 meter.
- **Returns**: Translation speed to track the target.
- [**More Info**](https://github.com/levifitzpatrick1/robitcode/blob/main/Documentation/File%20Specific/Complex%20Functions/trackTargetIDPosition(Integer%20targetID).md)
  
#### `driveToPosition2D(Pose2d targetPose)`

- **Description:** Moves the robot to the specified 2d Position on the field.
- **Returns:** Chassis speed to reach that destination.
- [**More Info**](https://github.com/levifitzpatrick1/robitcode/blob/main/Documentation/File%20Specific/Complex%20Functions/driveToPosition2D(Pose2d%20targetPose).md)
