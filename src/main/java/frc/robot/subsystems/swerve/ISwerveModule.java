package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public interface ISwerveModule

 {
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    public Rotation2d getCanCoder();

    public SwerveModuleState getState();

    public SwerveModulePosition getPosition();
    
    public int getModuleNumber(); 

    public void setModuleNumber(int moduleNumber);

    public CANSparkMax getDriveMotor();
    public CANSparkMax getAngleMotor();

}