package frc.robot.subsystems.swerve;

import frc.lib.math.GeometryUtils;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.text.BreakIterator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {


    public SwerveDriveOdometry swerveOdometry;
    public ISwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // WPILib
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


    public Swerve() {
        
       gyro = new Pigeon2(SwerveConstants.REV.pigeonID);
       // gyro.configFactoryDefault();
       Pigeon2Configuration PigeonConfig = new Pigeon2Configuration();
       gyro.getConfigurator().apply(PigeonConfig);
        
     

        mSwerveMods = new ISwerveModule[] {
            new SwerveMod(0, SwerveConstants.Swerve.Mod0.constants),
            new SwerveMod(1, SwerveConstants.Swerve.Mod1.constants),
            new SwerveMod(2, SwerveConstants.Swerve.Mod2.constants),
            new SwerveMod(3, SwerveConstants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(SwerveConfig.swerveKinematics, getYaw(), getModulePositions());
        zeroGyro();

        Logger.recordOutput("ModuleStates", getModuleStates());

            // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
                SwerveConfig.maxSpeed, // Max module speed, in m/s
                SwerveConfig.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
);
}


  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConfig.maxSpeed);

    for (int i = 0; i < mSwerveMods.length; i++) {
      //mSwerveMods[i].setTargetState(targetStates[i]);
      mSwerveMods[i].setDesiredState(targetStates[i], false);
    }
  }

  public ChassisSpeeds getSpeeds() {
    return SwerveConfig.swerveKinematics.toChassisSpeeds(getModuleStates());
  }
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // ChassisSpeeds.discretize(getSpeeds(), rotation)
        ChassisSpeeds desiredChassisSpeeds =
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(),
        rotation,
        getYaw())
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        SwerveModuleState[] swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConfig.maxSpeed);
        
        for(ISwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);
        
        for(ISwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    
    public Pose2d getPose() {
        Pose2d p =  swerveOdometry.getPoseMeters();
        return new Pose2d(-p.getX(),-p.getY(),  p.getRotation());
    }
    public void resetOdometry(Pose2d pose) {
        
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());
       
    }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(ISwerveModule mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }



    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(ISwerveModule mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(double degreeToSet) {
        if(SwerveConfig.invertGyro) {
            degreeToSet = -degreeToSet;
        }
        gyro.setYaw(degreeToSet);
        swerveOdometry.update(getYaw(), getModulePositions());  
    }

    public void zeroGyro() {  
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (SwerveConfig.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void periodic() {
       // SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());
        for(ISwerveModule mod : mSwerveMods) {
            // SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);    
            // SmartDashboard.putNumber("Abs encoder: " + mod.getModuleNumber(), mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("Rev Mod: " + mod.getModuleNumber() + "Drive Amp ", mod.getDriveMotor().getOutputCurrent());
            // SmartDashboard.putNumber("Rev Mod: " + mod.getModuleNumber() + "Azimuth Amp ", mod.getAngleMotor().getOutputCurrent());

           // SmartDashboard.putData("haha", mod.getMo);

         //  SmartDashboard.putNumber("Position error: ", )

         publisher.set(getModuleStates());

        }

       // SmartDashboard.putBoolean("Robot centric", RobotContainer.centric);
    }

  


    
}