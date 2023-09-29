// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.DriveCommands.ResetHeadingCmd;
import frc.robot.Commands.DriveCommands.SwerveJoystickCmd;
import frc.robot.Commands.VisionCommands.TrackTargetIDRotCmd;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  final Drivetrain drivetrain = new Drivetrain();
  
  private final Joystick driverController = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      drivetrain, 
      () -> driverController.getRawAxis(OIConstants.kXboxLeftXAxis),
      () -> driverController.getRawAxis(OIConstants.kXboxLeftYAxis),
      () -> driverController.getRawAxis(OIConstants.kXboxRightXAxis),
      () -> driverController.getRawButton(OIConstants.kXboxAButton)));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverController, OIConstants.kXboxBButton)
      .onTrue(new ResetHeadingCmd(drivetrain));

    new JoystickButton(driverController, OIConstants.kXboxXButton)
      .whileTrue(new TrackTargetIDRotCmd(drivetrain, 1));

      new JoystickButton(driverController, OIConstants.kXboxYButton)
      .whileTrue(new TrackTargetIDRotCmd(drivetrain, 2));
  }

 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
