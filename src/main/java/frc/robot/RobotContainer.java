// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Swerve swerve;

  private BooleanSupplier dampen;

  public RobotContainer() {
    swerve = new Swerve();

   /*  swerve.setDefaultCommand(
      new SwerveCommand(
        swerve, 
        () -> driverController.getLeftY(),
        () -> driverController.getLeftX(), 
        () -> driverController.getRightX(), 
        () -> false, 
        () -> dampen, 
        1)
    );*/
    configureBindings();
  }


  
  private void configureBindings() {
    //driverController.rightBumper().onTrue(
     // new InstantCommand(() -> dampen = true)
   // );
  }

  
}
