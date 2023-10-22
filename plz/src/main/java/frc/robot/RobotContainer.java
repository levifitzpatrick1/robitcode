// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.DriveCommands.DriveWithJoysticks;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.Mode;
import frc.robot.Subsystems.Drive.DriveWithIO;
import frc.robot.Subsystems.Drive.IOGyro;
import frc.robot.Subsystems.Drive.IOModuleSim;
import frc.robot.Subsystems.Drive.IOModuleSparkMAX;
import frc.robot.Subsystems.Drive.IOPigeon2;
import frc.robot.Util.Alert;


public class RobotContainer {
  private DriveWithIO drive;

  private XboxController driverController;

  private boolean isFieldCentric = true;

  public RobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_PHYSICAL:
        drive = new DriveWithIO(
          new IOPigeon2(),
          new IOModuleSparkMAX(3, "FL"),
          new IOModuleSparkMAX(4, "FR"),
          new IOModuleSparkMAX(2, "BL"),
          new IOModuleSparkMAX(1, "BR"));
          break;
      case ROBOT_SIMBOT:
        drive = new DriveWithIO(
          new IOGyro() {},
          new IOModuleSim(),
          new IOModuleSim(),
          new IOModuleSim(),
          new IOModuleSim());
        break;

      default:
        break;

      }
    }

    driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

    drive = drive != null ? drive :
      new DriveWithIO(
        new IOGyro() {},
        new IOModuleSim(),
        new IOModuleSim(),
        new IOModuleSim(),
        new IOModuleSim());


    if (Constants.tuningMode) {
      new Alert("Tuning Mode Enabled", Alert.AlertType.INFO).set(true);
    }

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
    () -> -driverController.getLeftY(),
    () -> -driverController.getLeftX(),
    () -> -driverController.getRightX(),
    () -> !isFieldCentric 
    ));
  }

}
