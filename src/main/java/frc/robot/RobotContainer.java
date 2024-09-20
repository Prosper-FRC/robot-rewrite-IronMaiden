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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.auto.Autonomous;
// import frc.robot.auto.DriveDistance;
// import frc.robot.commands.DriveCommands;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.GyroIO;
// import frc.robot.subsystems.drive.GyroIOPigeon2;
// import frc.robot.subsystems.drive.ModuleIO;
// import frc.robot.subsystems.drive.ModuleIOSim;
// import frc.robot.subsystems.drive.ModuleIOSparkMax;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// import frc.robot.auto.DriveDistance;

import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
// import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.arm.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final SendableChooser<Command> autoChooser;

    private final Joystick driver = new Joystick(Constants.kDriverControllerPort);
    public static boolean centric = false;

   /* Driver Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
	private final int rotationAxis = XboxController.Axis.kRightX.value;

  // private final int translationAxis = XboxController.Axis.kRightX.value;
	// private final int strafeAxis = XboxController.Axis.kLeftX.value;
	// private final int rotationAxis = XboxController.Axis.kLeftY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
   // private final PoseEstimator s_PoseEstimator = new PoseEstimator();



  // Subsystems

  // Controller
  private static final CommandXboxController driverController =
      new CommandXboxController(Constants.kDriverControllerPort);
  private static final CommandXboxController operatorController =
      new CommandXboxController(Constants.kOperatorControllerPort);

  private static Shooter shooter;
  private static Intake intake;
  private static Arm arm;

  public Command getAutonomousCommand() {

    PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    return AutoBuilder.followPath(path);
  }


  // Dashboard inputs

  

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> centric,
                () -> dampen.getAsBoolean(),
                () -> 1 //speed multiplier 
            )
        );

        shooter = new Shooter();
        intake = new Intake();
        arm = new Arm();

        autoChooser = AutoBuilder.buildAutoChooser();

     //   NamedCommands.registerCommand("straightPath", getAutonomousCommand());


        SmartDashboard.putData("Auto Chooser", autoChooser);


        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
  // private static final LEDs leds = new LEDs();
  // public static final Arm arm = new Arm(operator.getHID(), leds);

  // private static Intake intake;
  // private static Autonomous autonomous;

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  public Command ampButtonBinding() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setAmpSpeed()),
        new InstantCommand(() -> arm.goToAmpPos()),
        new WaitCommand(1.0),
        new InstantCommand(() -> intake.intake()));
  }

  public Command shootButtonBinding() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setSpeakerSpeed()),
        new InstantCommand(() -> arm.goToShootPos()),
        new WaitCommand(1.0),
        new InstantCommand(() -> intake.intake()));
        
  }
  

  public Command testShooterAndIntake() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> shooter.setSpeakerSpeed()),
        new InstantCommand(() -> intake.intake())
    );
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
    operatorController
        .leftBumper()
        .whileTrue( 
            new InstantCommand(
                () -> {
                  intake.intake();
                }))
            .onFalse(intake.retract());
        // .onFalse(new InstantCommand(
        //   () -> intake.retract()));



    // 'a' button outtakes note
    operatorController
        .rightBumper()
        .whileTrue(new InstantCommand(() -> intake.outtake()))
        .onFalse(new InstantCommand(() -> intake.zero()));

    // driver.y().onTrue(autonomous.SHOOT_MOBILITY());

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

    operatorController
        .povUp()
        .whileTrue(new InstantCommand(() -> shooter.setSpeedReverse()))
        .onFalse(
            new InstantCommand(
                () -> {
                  shooter.zero();
                 // intake.zero();
                }));

    operatorController
    .povDown()
    .whileTrue(new InstantCommand(() -> shooter.setSpeakerSpeed()))
    .onFalse(
        new InstantCommand(
            () -> {
                shooter.zero();
                //intake.zero();
            }
        )
    );

    operatorController
    .povLeft()
    .whileTrue(new InstantCommand(() -> testShooterAndIntake()))
    .onFalse(
        new InstantCommand(
            () -> {
                shooter.zero();
                intake.zero();
            }
        )
    );

    operatorController
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.goToClimbDownPos();
                }));

    operatorController
        .b()
        .whileTrue(ampButtonBinding())
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> arm.goToShootPos()),
                new InstantCommand(() -> intake.zero()),
                new InstantCommand(() -> shooter.zero())));

    operatorController
        .y()
        .whileTrue(
            new InstantCommand(
                () -> {
                  arm.goToClimbUpPos();
                }));

    operatorController
        .x()
        .whileTrue(shootButtonBinding())
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> arm.goToShootPos()),
                new InstantCommand(() -> intake.zero()),
                new InstantCommand(() -> shooter.zero())));

    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> driver.getRightX()));

    // driver.leftBumper().onTrue(autonomous.SHOOT_MOBILITY_LOAD());

    // driver.rightBumper().onTrue(leds.toggleBlue());

    // driver.rightTrigger().onTrue(new InstantCommand(() -> arm.climbOff()));

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

    /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        robotCentric.toggleOnTrue(new InstantCommand(() -> centric = !centric));

        //heading lock bindings
        // up.onTrue(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.d90)).onFalse(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        //     );
        // left.onTrue(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.d180)).onFalse(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        //     );
        // right.onTrue(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.d0)).onFalse(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        //     );
        // down.onTrue(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.d270)).onFalse(
        //     new InstantCommand(() -> States.driveState = States.DriveStates.standard)
        //     );
    
//         .b()
//         .whileTrue(ampButtonBinding())
//         .onFalse(
//             new ParallelCommandGroup(
//                 new InstantCommand(() -> arm.goToShootPos()),
//                 new InstantCommand(() -> intake.zero()),
//                 new InstantCommand(() -> shooter.zero())));
  
  //  driver.rightBumper().onTrue(leds.toggleBlue());

    driverController.rightTrigger().onTrue(new InstantCommand(() -> arm.climbOff()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return autoChooser.get();
  // }
  // public Command getAutonomousCommand() {
  //  // return autoChooser.get();
  // }
}