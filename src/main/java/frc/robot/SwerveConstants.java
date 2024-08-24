package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;


/**
 * This file comes with command robot projects, and is intended to contain
 * configuration information.
 * I think it would be prudent if this file only contained CanIDs, because it
 * is useful to have all the ids for the whole robot in one place.
 * other configuration goes into subsystem specific configuration files,
 * to make sure this one isn't cluttered.
 */
public final class SwerveConstants 
{
    public static final double stickDeadband = 0.1;
    public static final double limelightOffset = 3;
  

    public static final class REV
    {
        public static final int pigeonID = 41;

       
    
    }
    public static final class Swerve {
        /* Module Specific Constants */
    /* Front Left Module */
        public static final class Mod0 { 

            public static final int driveMotorID = 11;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 31;
            // 0.046143 + 0.5 for offset
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.473389 - 13.45 + 15.56 + 12.39); //
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Front Right Module */
        public static final class Mod1 { 
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 32;
            // -0.469482
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.022949 - 11.34 + 10.63 + 10.28);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Back Left Module */
        public static final class Mod2 { 
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 23;
            public static final int canCoderID = 33;
            // 0.415039
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.084229 - 330.82 + 151.7 + 180 + 151.44 + 180);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
         /* Back Right Module */
        public static final class Mod3 { 
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 24;
            public static final int canCoderID = 34;
            // -0.237549 + 0.5
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.254883 - 272.46 + 180 + 272.37 + 90);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
    }

     

    
    
    

   

}