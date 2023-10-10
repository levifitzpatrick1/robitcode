This method allows the robot to track a target while maintaining an X distance of 1 meter from it.

#### Parameters

- `targetID`: The ID of the target to track. Set to 99 to track any target.

#### Implementation Details

1. Retrieves the latest result from the front camera using `FrontCam.photonCamera.getLatestResult()`.
2. Checks if the result has targets and if the best target's ID matches the given `targetID` or if `targetID` is set to 99.
3. Retrieves the 3D transform from the camera to the target (`cameraToTarget`).
4. Extracts the X, Y, and rotational positions from `cameraToTarget`.
5. Uses PID controllers to calculate the desired velocities in the X (`vx`), Y (`vy`), and rotational (`rotspeed`) directions.
6. Converts these velocities to `ChassisSpeeds`.
7. Converts `ChassisSpeeds` to `SwerveModuleState[]` using kinematics.
8. Desaturates the wheel speeds if they exceed the maximum velocity.
9. Sets the module states using `setModuleStates(states)`.

#### Returns

- `ChassisSpeeds`: The calculated chassis speeds based on the PID controllers.