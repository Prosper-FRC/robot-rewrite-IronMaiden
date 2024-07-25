// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;

import javax.xml.stream.events.StartElement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveCommand extends Command {
  /** Creates a new SwerveCommand. */
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private DoubleSupplier speedDial;

    private PIDController rotationController;

  public SwerveCommand(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotOrientSup, DoubleSupplier speedDial) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);

    rotationController = new PIDController(0.01, 0, 0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(SwerveConstants.rotPIDTolerance);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotOrientSup;
    this.speedDial = speedDial;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = translationSup.getAsDouble();
    double strafeVal = strafeSup.getAsDouble();
    double  rotationVal = rotationSup.getAsDouble();
  
  switch(States.driveState){
            case d0:

                //heading lock
               rotationVal = rotationController.calculate(swerve.getYaw().getRadians(), Units.degreesToRadians(0));
                break;
            case d90:

                //heading lock
                rotationVal = rotationController.calculate(swerve.getYaw().getRadians(), Units.degreesToRadians(90));
                break;
            case d180:

                //heading lock
                rotationVal = rotationController.calculate(swerve.getYaw().getRadians(), Units.degreesToRadians(180));
                break;
            case d270:

                //heading lock
                rotationVal = rotationController.calculate(swerve.getYaw().getRadians(), Units.degreesToRadians(270));
                break;


            case standard:
            
                //normal
                rotationVal = rotationVal * SwerveConstants.maxAngularVelocity;
                break;
        }

        swerve.drive(
          new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed),
          rotationVal,
          robotCentricSup.getAsBoolean());
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
