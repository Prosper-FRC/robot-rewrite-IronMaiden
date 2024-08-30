// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.intake.Intake;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
 // private Intake intake;

  private CANSparkMax leftShootMotor;
  private CANSparkMax rightShootMotor;

  public Shooter(/*Intake intake*/) {
    // this.intake = intake;
    leftShootMotor = new CANSparkMax(ShooterConstants.k_leftShootMotorID, MotorType.kBrushless);
    rightShootMotor = new CANSparkMax(ShooterConstants.k_rightShootMotorID, MotorType.kBrushless);

    leftShootMotor.setInverted(ShooterConstants.k_isInverted);
    rightShootMotor.setInverted(ShooterConstants.k_isInverted);

    configure(leftShootMotor);
    configure(rightShootMotor);
  }

  // Set motor speeds to the speed needed to score Speaker
  public void setSpeakerSpeed() {
    leftShootMotor.set(ShooterConstants.k_shootSpeaker);
    rightShootMotor.set(ShooterConstants.k_shootSpeaker);
  }

  // Set motor speeds to the speed needed to score Amp, if different from Speaker
  public void setAmpSpeed() {
    leftShootMotor.set(ShooterConstants.k_shootAmp);
    rightShootMotor.set(ShooterConstants.k_shootAmp);
  }

  // Intake from source by reversing shooter motors
  public void setSpeedReverse() {
    leftShootMotor.set(-ShooterConstants.k_shootAmp);
    rightShootMotor.set(-ShooterConstants.k_shootAmp);
  }

  public void setSlowSpeedReverse() {
    leftShootMotor.set(-ShooterConstants.k_shootReverseSlow);
    rightShootMotor.set(-ShooterConstants.k_shootReverseSlow);
  }

  // Set motor speeds to zero
  public void zero() {
    leftShootMotor.set(ShooterConstants.k_shootZero);
    rightShootMotor.set(ShooterConstants.k_shootZero);
  }

  // Command to rev up shooter and run intake to shoot speaker
  public Command shootSpeaker() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setSpeakerSpeed()),
        new WaitCommand(ShooterConstants.k_waitTime),
        new InstantCommand(() -> System.out.println("Intaking...")));
  }

  // Command to rev up shooter and run intake to shoot amp
  public Command shootAmp() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setAmpSpeed()),
        new WaitCommand(ShooterConstants.k_waitTime),
        new InstantCommand(() -> System.out.println("Intaking...")));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left shooter current", leftShootMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left shooter temp", leftShootMotor.getMotorTemperature());

    SmartDashboard.putNumber("Right shooter current", rightShootMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right shooter temp", rightShootMotor.getMotorTemperature());
  }

  // Configure motors
  public void configure(CANSparkMax motor) {
    motor.setIdleMode(IdleMode.kCoast);
    motor.setSmartCurrentLimit(ShooterConstants.k_smartCurrentLimit);
    motor.setSecondaryCurrentLimit(50);
    motor.burnFlash();
    motor.clearFaults();
  }
}