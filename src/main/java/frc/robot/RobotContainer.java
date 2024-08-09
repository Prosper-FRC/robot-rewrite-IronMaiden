// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.auto.DriveDistance;
import frc.robot.subsystems.arm.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  // Controller
  private static final CommandXboxController driver =
      new CommandXboxController(Constants.kDriverControllerPort);
  private static final CommandXboxController operator =
      new CommandXboxController(Constants.kOperatorControllerPort);

  
  public static final Arm arm = new Arm(operator.getHID());

 

  // Dashboard inputs
  

  public Command ampButtonBinding() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(new InstantCommand(() -> System.out.println("Shooter running at amp speed..."))),
        new InstantCommand(() -> arm.goToAmpPos()),
        new WaitCommand(1.0),
        new InstantCommand(() -> System.out.println("Intakng...")));
  }

  public Command shootButtonBinding() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(new InstantCommand(() -> System.out.println("Shooter running at speaker speed..."))),
        new InstantCommand(() -> arm.goToShootPos()),
        new WaitCommand(1.0),
        new InstantCommand(() -> System.out.println("Intaking...")));
  }
  ;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

   


    // Manual:
    // arm.setDefaultCommand(new ArmCommand(() -> operator.getRightY(), arm));

    

    

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  // -------------------------------------------------------------------[Button
  // Bindings]-----------------------------------------------------------------------

  private void configureButtonBindings() {

    // right bumper intakes note and retracts to move note away from shooter wheels
    // operator
    //     .leftBumper()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               intake.intake();
    //             }))
    //     .onFalse(intake.retract());

    // 'a' button outtakes note
    // operator
    //     .rightBumper()
    //     .whileTrue(new InstantCommand(() -> intake.outtake()))
    //     .onFalse(new InstantCommand(() -> intake.zero()));


    /*// Left bumper shoots Speaker, now shootSpeaker also includes moving intake also.
    operator
        .leftTrigger()
        .whileTrue(shooter.shootSpeaker())
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.zero();
                  intake.zero();
                }));

    // Left joystick CLICK shoots amp, also moves intake using one button
    operator
        .rightTrigger()
        .whileTrue(shooter.shootAmp())
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.zero();
                  intake.zero();
                }));*/

    // operator
    //     .povUp()
    //     .whileTrue(new InstantCommand(() -> shooter.setSpeedReverse()))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               shooter.zero();
    //               intake.zero();
    //             }));

    operator
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.goToClimbDownPos();
                }));

    operator
        .b()
        .whileTrue(ampButtonBinding())
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> arm.goToShootPos()),
                new InstantCommand(() -> System.out.println("Intaking...")),
                new InstantCommand(() -> System.out.println("Outtaking..."))));
// operator
//         .b()
//         .whileTrue(ampButtonBinding())
//         .onFalse(
//             new ParallelCommandGroup(
//                 new InstantCommand(() -> arm.goToShootPos()),
//                 new InstantCommand(() -> intake.zero()),
//                 new InstantCommand(() -> shooter.zero())));
    operator
        .y()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.goToClimbUpPos();
                }));

    operator
        .x()
        .whileTrue(shootButtonBinding())
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> arm.goToShootPos()),
                new InstantCommand(() -> System.out.println("Intaking...")),
                new InstantCommand(() -> System.out.println("Outtaking..."))));

  

  //  driver.rightBumper().onTrue(leds.toggleBlue());

    driver.rightTrigger().onTrue(new InstantCommand(() -> arm.climbOff()));

   // driver.a().onTrue(new InstantCommand(() -> Drive.resetGyro()));

    /*
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller.b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.a()
        .whileTrue(
            Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));
    */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return autoChooser.get();
  // }
}