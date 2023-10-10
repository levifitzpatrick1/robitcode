This method drives the robot to a specified 2D position using PID control.

#### Parameters

- `targetPose`: The target position and orientation as a `Pose2d` object.

#### Implementation Details

1. Retrieves the current estimated position of the robot using `positionEstimator.getEstimatedPosition()`.
2. Calculates the errors in the X, Y, and rotational directions.
3. Uses PID controllers to calculate the desired velocities in the X (`vx`), Y (`vy`), and rotational (`rot`) directions.
4. Converts these velocities to `ChassisSpeeds`.
5. Converts `ChassisSpeeds` to `SwerveModuleState[]` using kinematics.
6. Desaturates the wheel speeds if they exceed the maximum velocity.
7. Sets the module states using `setModuleStates(states)`.

#### Returns

- `ChassisSpeeds`: The calculated chassis speeds based on the PID controllers.