## Overview

The `SwerveModule` class is responsible for controlling a single swerve module on the robot. It includes methods for getting the state of the module, setting the desired state, and stopping the module.

## Class Members

### Private Members

- `driveMotor`: Controls the drive motor.
- `turnMotor`: Controls the turning motor.
- `driveEncoder`: Encoder for the drive motor.
- `turnEncoder`: Encoder for the turning motor.
- `turningProfiledPIDController`: PID controller for turning.
- `drivePIDController`: PID controller for driving.
- `absoluteEnCoder`: Absolute encoder for the module.
- `absoluteEnCoderOffset`: Offset for the absolute encoder.
- `absoluteEnCoderReversed`: Flag for reversing the absolute encoder.
- `constants`: Module-specific constants.

## Constructor

### `SwerveModule(int moduleID, String moduleLocation)`

Initializes a new `SwerveModule` with the given `moduleID` and `moduleLocation`.

- `moduleID`: Unique ID of the module.
- `moduleLocation`: Location of the module on the robot (FL, FR, BL, BR).

## Methods

### Getters

#### `getDrivePosition()`

Returns the drive position in meters based on the encoder's conversion factor.

#### `getDriveVelocity()`

Returns the drive velocity in meters per second based on the encoder's conversion factor.

#### `getTurnPosition()`

Returns the turn position in radians based on the encoder's conversion factor.

#### `getTurnVelocity()`

Returns the turn velocity in radians per second based on the encoder's conversion factor.

#### `getAbsolutePositionRadians()`

Returns the absolute position in radians of the encoder.

#### `getAbsolutePositionDegrees()`

Returns the absolute position in degrees of the encoder.

#### `getModulePosition()`

Returns the position of the module as a `SwerveModulePosition`.

#### `getState()`

Returns the state of the module as a `SwerveModuleState`.

### Commands

#### `resetEncoders()`

Resets the drive and turn encoders to their initial positions.

#### `setDesiredState(SwerveModuleState state)`

Sets the desired state of the module. Stops the module if the speed is below a certain threshold.

- `state`: Desired state of the module as a `SwerveModuleState`.

#### `stop()`

Stops the drive and turn motors by setting their speeds to zero.

#### `getSpeed()`

Returns the maximum speed of the module.