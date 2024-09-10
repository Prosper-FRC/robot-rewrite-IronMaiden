// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax indexerMotor;

  public IntakeSubsystem() {
    indexerMotor = new CANSparkMax(15, MotorType.kBrushless);
  }

  // Set motor speed to intake note
  public void intake() {
    indexerMotor.set(0.75);
  }

  // Set motor speed to outtake note
  public void outtake() {
    indexerMotor.set(-0.5);
  }

  // Set motor speed to zero
  public void zero() {
    indexerMotor.set(0.0);
  }

  // Command to pull back note away from shooter wheels when intake button is released
//   public Command retract() {
//     return new SequentialCommandGroup(
//         new InstantCommand(() -> outtake()),
//         new WaitCommand(IntakeConstants.k_waitTime),
//         new InstantCommand(() -> zero()));
//   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Amps", indexerMotor.getOutputCurrent());
  }

  // motor configurations
  public void configure() {
    indexerMotor.setIdleMode(IdleMode.kBrake);
    indexerMotor.setSmartCurrentLimit(40);
    // indexerMotor.burnFlash();
    indexerMotor.clearFaults();
  }
}