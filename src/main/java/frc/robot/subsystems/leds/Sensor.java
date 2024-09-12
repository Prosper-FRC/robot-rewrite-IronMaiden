// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.intake.Intake;

public class Sensor extends SubsystemBase {
  /** Creates a new Sensor. */
  // public static boolean noteDetected = false;

  private static DigitalInput sensor = null;
  private static LEDs led;
 // private static Intake intake;

  public Sensor(LEDs ledIn) {
    sensor = new DigitalInput(0);
    led = ledIn;
  }

  public boolean isDetected() {
   // System.out.println("Note detected");
    return !sensor.get();
  }

  public void getNoteDetected() {
    if (isDetected()) {
      led.setLEDsPurple();
      System.out.println("Note detected");
    }
    else {
      led.ladyChaser();
    }
  }

//   public static void stopIntake() {
//     if (isDetected()) {
//       intake.zero();
//     }
//   }

  @Override
  public void periodic() {
    // noteDetected = isDetected();
    // getNoteDetected();
    // if (noteDetected) {
    //   led.blinkCommand().withTimeout(2);
    // }

    //isDetected();
    getNoteDetected();

  }
}