package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    // in order to fix the errors not displaying on the newly cloned repo, reboot VS Code

    public static final double wheelCircumference = 12.5663706144;
    public static final double trackWidth = Units.inchesToMeters(25);
    public static final double wheelBase = Units.inchesToMeters(25);
    public static final double driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double azimuthGearRatio = 150.0 / 7.0;

     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double driveRevToMeters = wheelCircumference / driveGearRatio;
    public static final double driveRPMToMetersPerSecond = driveRevToMeters / 60;
    public static final double DegreesPerTurnRotation = 360 / azimuthGearRatio;

    public static boolean invertGyro = false; 

    public static final double driveMotorP = 0.0;
    public static final double driveMotorI = 0.0;
    public static final double driveMotorD = 0.0;

    public static final double driveMaxVel = 0.0;
    public static final double driveMaxAccel = 0.0;

    public static final double driveMotorF = 0.0;

    public static final double azimuthMotorP = 0.0;
    public static final double azimuthMotorI = 0.0;
    public static final double azimuthMotorD = 0.0;

    public static final double rotPIDTolerance = 1;

    public static final double azimuthMaxVel = 0.0;
    public static final double azimuthMaxAccel = 0.0;

    public static final double azimuthMotorF = 0.0;

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode azimuthIdleMode = IdleMode.kBrake;

    // For PID controller output range
    public static final double drivePower = 1.0;
    public static final double azimuthPower = 0.9;

    public static final boolean driveMotorInvert = false;
    public static final boolean azimuthMotorInvert = true;

    public static final int driveCurrentLimit = 40; // 60
    public static final int azimuthCurrentLimit = 40; // 30

    // In meters per second:
    public static final double driveBaseRadius =  Math.hypot(trackWidth / 2.0, trackWidth / 2.0);
    public static final double maxSpeed = Units.feetToMeters(17.1); // 4.0
    public static final double maxAngularVelocity = maxSpeed / driveBaseRadius;

    public static final int gyroID = 0;


    

}
