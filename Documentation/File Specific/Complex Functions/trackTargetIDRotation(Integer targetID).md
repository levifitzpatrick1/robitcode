This method allows the robot to track the rotation of a target with a given ID.

#### Parameters

- `targetID`: The ID of the target to track. Set to 99 to track any target.

#### Implementation Details

1. Retrieves the latest result from the front camera using `FrontCam.photonCamera.getLatestResult()`.
2. Checks if the result has targets and if the best target's ID matches the given `targetID` or if `targetID` is set to 99.
3. Retrieves the best target's ID and yaw angle, converting the yaw to radians.
4. Uses an angular PID controller to calculate the desired rotational speed (`rotspeed`).
5. Displays the found target ID, yaw angle, and calculated rotational speed on the Smart Dashboard.

#### Returns

- `double`: The calculated rotational speed based on the PID controller.