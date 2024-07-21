package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
// import com.fasterxml.jackson.databind.Module;

import frc.robot.subsystems.swerve.Module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    
    public SwerveDriveOdometry swerveOdometry;
    public Module[] swerveMods;
    public Pigeon2 gyro;


public Swerve() {
    gyro = new Pigeon2(SwerveConstants.gyroID);
    
    swerveMods = new Module[] {
        //public Module(int moduleNumber, int azimuthMotorID, int driveMotorID, int cancoderID, Rotation2d angleOffset) {

        new Module(0, 0, 0, 0, new Rotation2d(0)),
        new Module(1, 0, 0, 0, new Rotation2d(0)),
        new Module(2, 0, 0, 0, new Rotation2d(0)),
        new Module(3, 0, 0, 0, new Rotation2d(0)),
    };

    swerveOdometry = new SwerveDriveOdometry(null, null, null);
}
}
