package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.RevSwerveModuleConstants;

public class SwerveConfig {
    public static final double stickDeadband = 0.1;
    public static final double limelightOffset = 3;
  

    public static final class REV
    {
        public static final int pigeonID = 13;

       
    
    }
    public static final class Swerve {
        /* Module Specific Constants */
    /* Front Left Module */
        public static final class Mod0 { 

            public static final int driveMotorID = 11;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.046142 + 0.5); //Rotation2d.fromDegrees(37.7);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Front Right Module */
        public static final class Mod1 { 
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations((-0.469482));
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Back Left Module */
        public static final class Mod2 { 
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 23;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.415039);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
         /* Back Right Module */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.237549 + 0.5);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
    }

     

    
    
    

   
}
