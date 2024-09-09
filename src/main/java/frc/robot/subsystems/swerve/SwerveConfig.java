package frc.robot.subsystems.swerve;

//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.IdleMode;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;


public class SwerveConfig 
{
    
    public CANcoderConfiguration canCoderConfig;

    //
    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    // Drive and angle power are the output ranges for their respective
    // PID controllers
    public static final double drivePower = 1.0; // 0.1
    public static final double anglePower = 0.9; // 0.1


    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =  
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(25.0); 
    public static final double wheelBase = Units.inchesToMeters(25.0); 
    public static final double wheelCircumference = chosenModule.wheelCircumference;


    public static final double maxModuleSpeed = 4.5; // M/S
    
    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    // encoder setup
    // meters per rotation
    public static final double driveRevToMeters =  wheelCircumference / driveGearRatio;
    public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60 ;
    // the number of degrees that a single rotation of the turn motor turns the wheel.
    public static final double DegreesPerTurnRotation = 360/angleGearRatio;

    
    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 50;
    public static final int anglePeakCurrentLimit = 40; // Not in use
    public static final double anglePeakCurrentDuration = 0.1; // Not in use
    public static final boolean angleEnableCurrentLimit = true; // Not in use

    public static final int driveContinuousCurrentLimit = 50; 
    public static final int drivePeakCurrentLimit = 60; // Not in use
    public static final double drivePeakCurrentDuration = 0.1; // Not in use
    public static final boolean driveEnableCurrentLimit = true; // Not in use

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.2; // 6.0, 0.1
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.15;

    // public static final double angleKP = 0.0; // 6.0, 0.1
    // public static final double angleKI = 0.0;
    // public static final double angleKD = 0.0;

    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.035; 
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.05; // try 0.1 or 0.13 later on 


    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.1); 
    public static final double driveKV = (0.1);
    public static final double driveKA = (0.05);

    /* Swerve Profiling Values */
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, trackWidth / 2.0);
    /** Meters per Second */
    public static final double maxSpeed = Units.feetToMeters(17.1); // 4.0
    /** Radians per Second */
    public static final double maxAngularVelocity = maxSpeed / driveBaseRadius;
   

 
 
   

    public SwerveConfig()
    {
        canCoderConfig = new CANcoderConfiguration();
        // canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // canCoderConfig.sensorDirection = canCoderInvert;
        // canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    }
}

