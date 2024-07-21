package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private CANSparkMax azimuthMotor;
    private CANSparkMax driveMotor;

    private SparkPIDController driveController = driveMotor.getPIDController();
    
    private SparkPIDController azimuthController = azimuthMotor.getPIDController();

    private CANcoder CANCoder;
    private RelativeEncoder relAzimuthEncoder;
    private RelativeEncoder relDriveEncoder;

    private int azimuthMotorID;
    private int driveMotorID;
    private int cancoderID;

    public Module(int moduleNumber, int azimuthMotorID, int driveMotorID, int cancoderID, Rotation2d angleOffset) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        this.azimuthMotorID = azimuthMotorID;
        this.driveMotorID = driveMotorID;
        this.cancoderID = cancoderID;

        azimuthMotor = new CANSparkMax(azimuthMotorID, MotorType.kBrushless);
        configureAzimuth();

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        configureDrive();

        CANCoder = new CANcoder(cancoderID);

    }

    private void configureDrive() {
        driveMotor.restoreFactoryDefaults();
        driveController.setP(SwerveConstants.driveMotorP);
        driveController.setI(SwerveConstants.driveMotorI);
        driveController.setD(SwerveConstants.driveMotorD);
        driveController.setFF(SwerveConstants.driveMotorF);

        // Ask about first parameter for output range
        driveController.setOutputRange(-SwerveConstants.drivePower, SwerveConstants.drivePower);
        driveMotor.setSmartCurrentLimit(SwerveConstants.driveCurrentLimit);
        driveMotor.setInverted(SwerveConstants.driveMotorInvert);
        driveMotor.setIdleMode(SwerveConstants.driveIdleMode);

        driveMotor.burnFlash();
        driveMotor.clearFaults();
    }

    private void configureAzimuth() {
        azimuthMotor.restoreFactoryDefaults();
        azimuthController.setP(SwerveConstants.azimuthMotorP);
        azimuthController.setI(SwerveConstants.azimuthMotorI);
        azimuthController.setD(SwerveConstants.azimuthMotorD);
        azimuthController.setFF(SwerveConstants.azimuthMotorF);

        // Ask about first parameter for output range
        azimuthController.setOutputRange(-SwerveConstants.azimuthPower, SwerveConstants.azimuthPower);
        azimuthMotor.setSmartCurrentLimit(SwerveConstants.azimuthCurrentLimit);
        azimuthMotor.setInverted(SwerveConstants.driveMotorInvert);
        azimuthMotor.setIdleMode(SwerveConstants.driveIdleMode);

        azimuthMotor.burnFlash();
        azimuthMotor.clearFaults();
    }

    private void configureEncoder() {
        relDriveEncoder = driveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        relDriveEncoder.setPositionConversionFactor(SwerveConstants.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(SwerveConstants.driveRPMToMetersPerSecond);

        relAzimuthEncoder = azimuthMotor.getEncoder();
        relAzimuthEncoder.setPositionConversionFactor(SwerveConstants.DegreesPerTurnRotation);
        relAzimuthEncoder.setVelocityConversionFactor(SwerveConstants.DegreesPerTurnRotation / 60);

        resetToAbsolute();

    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState);

        if(driveMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+driveMotor.getDeviceId(), false);
        }

        if(azimuthMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+azimuthMotor.getDeviceId(), false);
        }

    } 

    private void setSpeed(SwerveModuleState desiredState) {

        // double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
        // driveMotor.set(percentOutput);

        double velocity = desiredState.speedMetersPerSecond;

        driveController.setReference(velocity, ControlType.kVelocity);

    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    private void resetToAbsolute() {
        double absolutePosition = getCANCoder().getDegrees() - angleOffset.getDegrees();

        relAzimuthEncoder.setPosition(absolutePosition);
    }

    private Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(CANCoder.getAbsolutePosition().getValueAsDouble());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(relAzimuthEncoder.getPosition());
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = desiredState.angle;
        
        // This is to stop motor movement if the speed of the module is less than 1% the max speed to reduce jittering -
        // however, it may cause issues in the future. Where exactly are we supposed to call .set to reenable movement?
        if(Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01)) 
        {
            azimuthMotor.stopMotor();

        }


        double degPIDReference = angle.getDegrees();

        azimuthController.setReference(degPIDReference, ControlType.kPosition);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(relDriveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            relDriveEncoder.getPosition(), 
            getAngle()
        );
    }

    
}
