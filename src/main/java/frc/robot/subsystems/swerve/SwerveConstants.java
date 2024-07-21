package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    
    public static final double wheelCircumference = 0.0;

    public static final double driveGearRatio = 0.0;
    public static final double azimuthGearRatio = 0.0;

    public static final double driveRevToMeters = wheelCircumference / driveGearRatio;
    public static final double driveRPMToMetersPerSecond = driveRevToMeters / 60;
    public static final double DegreesPerTurnRotation = 360 / azimuthGearRatio;

    public static final double driveMotorP = 0.0;
    public static final double driveMotorI = 0.0;
    public static final double driveMotorD = 0.0;

    public static final double driveMaxVel = 0.0;
    public static final double driveMaxAccel = 0.0;

    public static final double driveMotorF = 0.0;

    public static final double azimuthMotorP = 0.0;
    public static final double azimuthMotorI = 0.0;
    public static final double azimuthMotorD = 0.0;

    public static final double azimuthMaxVel = 0.0;
    public static final double azimuthMaxAccel = 0.0;

    public static final double azimuthMotorF = 0.0;

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode azimuthIdleMode = IdleMode.kBrake;

    // For PID controller output range
    public static final double drivePower = 1.0;
    public static final double azimuthPower = 0.9;

    public static final boolean driveMotorInvert = false;
    public static final boolean azimuthMotorInvert = false;

    public static final int driveCurrentLimit = 60;
    public static final int azimuthCurrentLimit = 30;

    // In meters per second:
    public static final double maxSpeed = 0.0; // 4.0

    public static final int gyroID = 0;

    public static final double trackWidth = Units.inchesToMeters(0.0);
    public static final double wheelBase = Units.inchesToMeters(0.0);
    public static final double nothing = 0;

}
