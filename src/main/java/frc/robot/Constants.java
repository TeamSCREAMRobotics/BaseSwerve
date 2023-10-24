package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants.Module;

/**
 * A class for constants used in various places in the project.
 */
public final class Constants{

    public record MotionMagicConstants(double cruiseVelocity, double acceleration, int sCurveStrength){}

    /* Robot loop time */
    public static final double LOOP_TIME_SEC = 0.02;
    public static final double LOOP_TIME_HZ = 1 / LOOP_TIME_SEC;

    public static final class Ports {
        /** Possible CAN bus strings are:
         *   • "rio" for the native roboRIO CAN bus 
         *   • CANivore name or serial number 
         *   • "*" for any CANivore seen by the program
         */
        public static final String CAN_BUS_NAME = "canivore"; // TODO ROBOT SPECIFIC

        /* Pigeon2 */
        public static final int PIGEON_ID = 0; // TODO ROBOT SPECIFIC
    }

    
    public static final class ShuffleboardConstants {

        /* For live updating values like PID from Shuffleboard */
        public static final boolean UPDATE_SWERVE = false;
    }


    public static final class SwerveConstants {

        /* Drivebase Constants */
        // TODO ROBOT SPECIFIC
        public static final double TRACK_WIDTH = 0.50165; // Distance from left wheels to right wheels/vice versa
        public static final double WHEEL_BASE = 0.57531; // Distance from front wheels to back wheels/vice versa

        /* Gyro Constants */
        public static final boolean GYRO_INVERT = false; // TODO ALWAYS ENSURE GYRO READS CCW+ CW-

        /* Swerve Kinematics */
        public static final double MAX_SPEED = 5.7349; // m/s
        public static final double MAX_ANGULAR_VELOCITY = 8.0; // rad/s

        /* Selected Module Constants */
        // TODO ROBOT SPECIFIC
        public static final COTSFalconSwerveConstants MODULE_TYPE = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3); 

        /* Swerve Kinematics */
        // No need to ever change this unless there are more than four modules.
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        /** Selected Modules */
        // Use this if there are multiple sets of modules.
        // Set each location's constants to their corresponding module constants.
        public static final Module FRONT_LEFT =  new Module(0, ModuleConstants.MODULE_0);
        public static final Module FRONT_RIGHT = new Module(1, ModuleConstants.MODULE_1);
        public static final Module BACK_LEFT =   new Module(2, ModuleConstants.MODULE_2);
        public static final Module BACK_RIGHT =  new Module(3, ModuleConstants.MODULE_3);

        /* PID Controllers */
        /* Swerve Heading Correction */
        public static final PIDController HEADING_CORRECT_CONTROLLER = new PIDController(0.0, 0.0, 0.0);

        /* PathPlanner Constants */
        public static final PIDConstants PATH_TRANSLATION_CONSTANTS = new PIDConstants(0.0, 0.0, 0.0); // TODO ROBOT SPECIFIC
        public static final PIDConstants PATH_ROTATION_CONSTANTS = new PIDConstants(0.0, 0.0, 0.0);

        public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                PATH_TRANSLATION_CONSTANTS, 
                PATH_ROTATION_CONSTANTS, 
                MAX_SPEED, 
                new Translation2d(SwerveConstants.WHEEL_BASE/2, SwerveConstants.TRACK_WIDTH/2).getDistance(new Translation2d()), 
                new ReplanningConfig()
        );

        
        public static final class DriveConstants {
            /* Gear Ratio */
            public static final double GEAR_RATIO = MODULE_TYPE.driveGearRatio;

            /* Neutral Mode */
            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

            /* Motor Invert */
            public static final InvertedValue MOTOR_INVERT = MODULE_TYPE.driveMotorInvert;

            /* Current Limit Constants */
            public static final int SUPPLY_CURRENT_LIMIT = 35;
            public static final int SUPPLY_CURRENT_THRESHOLD = 60;
            public static final double SUPPLY_TIME_THRESHOLD = 0.1;
            public static final boolean CURRENT_LIMIT_ENABLE = true;

            /* Ramps Constants */
            public static final double OPEN_LOOP_RAMP = 0.25;
            public static final double CLOSED_LOOP_RAMP = 0.0;

            /* PID Constants */
            public static final double KP = 0.05; // TODO ROBOT SPECIFIC
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KF = 0.0;
            public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(KP, KI, KD, KF);

            /* Feedforward Constants */
            public static final double KS = (0.32 / 12); // TODO ROBOT SPECIFIC
            public static final double KV = (1.51 / 12);
            public static final double KA = (0.27 / 12);
        }


        public static final class SteerConstants {
            /* Gear Ratio */
            public static final double GEAR_RATIO = MODULE_TYPE.steerGearRatio;

            /* Motor Invert */
            public static final InvertedValue MOTOR_INVERT = MODULE_TYPE.steerMotorInvert;

            /* Neutral Modes */
            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast; // TODO CHANGE TO BRAKE AFTER MEASURING OFFSETS

            /* Current Limits */
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final int SUPPLY_CURRENT_THRESHOLD = 40;
            public static final double SUPPLY_TIME_THRESHOLD = 0.1;
            public static final boolean CURRENT_LIMIT_ENABLE = true;        

            /* PID */
            public static final double KP = MODULE_TYPE.steerKP; 
            public static final double KI = MODULE_TYPE.steerKI;
            public static final double KD = MODULE_TYPE.steerKD;
            public static final double KF = MODULE_TYPE.steerKF;
            public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(KP, KI, KD, KF);
        }


        public static class ModuleConstants{

            public record SwerveModuleConstants(int driveMotorID, int steerMotorID, int encoderID, Rotation2d angleOffset){}
            public record Module(int number, SwerveModuleConstants constants){
                public String toString(){
                    switch(number){
                        case 0:
                        return "FRONT_LEFT";
                        case 1:
                        return "FRONT_RIGHT";
                        case 2:
                        return "BACK_LEFT";
                        case 3:
                        return "BACK_RIGHT";
                    }
                    return null;
                }
            }
            
            /* Front Left */
            public static final SwerveModuleConstants MODULE_0 = new SwerveModuleConstants(
                23,           
                24, 
                8, 
                Rotation2d.fromDegrees(294.961)); // TODO ROBOT SPECIFIC

            /* Front Right */
            public static final SwerveModuleConstants MODULE_1 = new SwerveModuleConstants(
                13, 
                14, 
                3, 
                Rotation2d.fromDegrees(310.166)); // TODO ROBOT SPECIFIC

            /* Back Left */
            public static final SwerveModuleConstants MODULE_2 = new SwerveModuleConstants(
                19, 
                20, 
                6, 
                Rotation2d.fromDegrees(163.037)); // TODO ROBOT SPECIFIC

            /* Back Right */
            public static final SwerveModuleConstants MODULE_3 = new SwerveModuleConstants(
                17, 
                18, 
                5, 
                Rotation2d.fromDegrees(45.176)); // TODO ROBOT SPECIFIC
        }
    }
}
