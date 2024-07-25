package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
// import com.fasterxml.jackson.databind.Module;
// import frc.lib.math.GeometryUtils;

import frc.robot.lib.GeometryUtils;
import frc.robot.subsystems.swerve.Module;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    
    public static SwerveDriveOdometry swerveOdometry;
    public Module[] swerveMods;
    public Pigeon2 gyro;

    private static boolean sniperMode = false;

public Swerve() {
    gyro = new Pigeon2(SwerveConstants.gyroID);
    
    swerveMods = new Module[] {
        //public Module(int moduleNumber, int azimuthMotorID, int driveMotorID, int cancoderID, Rotation2d angleOffset) {

        new Module(0, 0, 0, 0, new Rotation2d(0)),
        new Module(1, 0, 0, 0, new Rotation2d(0)),
        new Module(2, 0, 0, 0, new Rotation2d(0)),
        new Module(3, 0, 0, 0, new Rotation2d(0)),
    };

    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
    zeroGyro();

}

private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME = 0.02;
    Pose2d futureRobotPose = 
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME)
        );
    //Twist2d log = Pose2d.log(futureRobotPose);
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);

    ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME,
                twistForPose.dy / LOOP_TIME,
                twistForPose.dtheta / LOOP_TIME);
        return updatedSpeeds;
}

public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    ChassisSpeeds desiredChassisSpeeds = 
    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
    : new ChassisSpeeds(
        translation.getX(),
        translation.getY(),
        rotation
    );



    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (Module mod : swerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()]);
    }
}

public void setSniperMode() {
    sniperMode = true;
}

public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (Module mod : swerveMods) {
        positions[mod.getModuleNumber()] = mod.getPosition();
    }
    return positions;
}

public Rotation2d getYaw() {
    // return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    if (SwerveConstants.invertGyro) {
        return Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble());
    }

    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

public void zeroGyro() {
    gyro.setYaw(0);
    swerveOdometry.update(getYaw(), getModulePositions());
}

}
