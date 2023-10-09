package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.COTSFalconSwerveConstants;

/**
 * A class for constants used in various places in the project.
 */
public final class Constants{

    public record MotionMagicConstants(double cruiseVelocity, double acceleration, int sCurveStrength){}

    public static final class Ports {
        /* CANivore */
        public static final String CANIVORE_BUS_NAME = "canivore"; // TODO DELETE IF ROBOT DOES NOT USE A CANivore

        /* Pigeon2 */
        public static final int PIGEON_ID = 0;
    }

    
    public static final class ShuffleboardConstants {
        /* Adds additional tabs for debugging */
        public static final boolean INCLUDE_DEBUG_TABS = true;

        /* For live updating values like PID from Shuffleboard */
        public static final boolean UPDATE_SWERVE = false;
        
    }


    public static final class SwerveConstants {

        public static final double TRACK_WIDTH = 0.0; // Distance from left wheels to right wheels/vice versa
        public static final double WHEEL_BASE = 0.0; // Distance from front wheels to back wheels/vice versa

        /* Selected Module Constants */ // TODO ROBOT SPECIFIC
        public static final COTSFalconSwerveConstants CHOSEN_MODULE = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3); 
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;
        public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.CANcoderInvert;

        /* Gyro Constants */
        public static final boolean GYRO_INVERT = false; // TODO ALWAYS ENSURE GYRO READS CCW+ CW-

        /* Swerve Profiling Constants */ // TODO ROBOT SPECIFIC
        public static final double MAX_SPEED = 0.0; // m/s
        public static final double MAX_ANGULAR_VELOCITY = 0.0; // rad/s

        /* PID Controllers */
        /* PathPlanner */
        public static final PIDController PATH_TRANSLATION_CONTROLLER = new PIDController(0.0, 0.0, 0.0); // TODO ROBOT SPECIFIC
        public static final PIDController PATH_ROTATION_CONTROLLER = new PIDController(0.0, 0.0, 0.0);
        
        /* Swerve angle holding */
        public static final PIDController SWERVE_HOLD_CONTROLLER = new PIDController(0.0, 0.0, 0.0);

        /* Swerve Kinematics
         * No need to ever change this unless there is more than four modules.
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));


        public static final class DriveConstants {
            /* Gear Ratio */
            public static final double GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;

            /* Neutral Mode */
            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

            /* Motor Invert */
            public static final InvertedValue MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

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


        public static final class AngleConstants {
            /* Gear Ratio */
            public static final double GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

            /* Motor Invert */
            public static final InvertedValue MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;

            /* Neutral Modes */
            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast; // TODO CHANGE TO BRAKE AFTER MEASURING OFFSETS

            /* Current Limits */
            public static final int SUPPLY_CURRENT_LIMIT = 25;
            public static final int SUPPLY_CURRENT_THRESHOLD = 40;
            public static final double SUPPLY_TIME_THRESHOLD = 0.1;
            public static final boolean CURRENT_LIMIT_ENABLE = true;        

            /* PID */
            public static final double KP = CHOSEN_MODULE.angleKP; 
            public static final double KI = CHOSEN_MODULE.angleKI;
            public static final double KD = CHOSEN_MODULE.angleKD;
            public static final double KF = CHOSEN_MODULE.angleKF;
            public static final ScreamPIDConstants ANGLE_PID_CONSTANTS = new ScreamPIDConstants(KP, KI, KD, KF);
        }


        public static class ModuleConstants{

            public record SwerveModuleConstants(int driveMotorID, int angleMotorID, int encoderID, Rotation2d angleOffset){}

            /** 
             * Use this if you have multiple sets of modules.
             * In our case, this is useful for hotswapping modules.
             * Set each location's constants to their corresponding module.
             */
            public static enum Modules{
                FRONT_LEFT(MODULE_0), 
                FRONT_RIGHT(MODULE_1), 
                BACK_LEFT(MODULE_2), 
                BACK_RIGHT(MODULE_3);

                private SwerveModuleConstants constants;

                private Modules(SwerveModuleConstants constants){
                    this.constants = constants;
                }

                public SwerveModuleConstants getAssociated(){
                    return constants;
                }
            }
            
            /* Front Left */
            public static final SwerveModuleConstants MODULE_0 = new SwerveModuleConstants(
                23,           
                24, 
                8, 
                Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC

            /* Front Right */
            public static final SwerveModuleConstants MODULE_1 = new SwerveModuleConstants(
                13, 
                14, 
                3, 
                Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC

            /* Back Left */
            public static final SwerveModuleConstants MODULE_2 = new SwerveModuleConstants(
                19, 
                20, 
                6, 
                Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC

            /* Back Right */
            public static final SwerveModuleConstants MODULE_3 = new SwerveModuleConstants(
                17, 
                18, 
                5, 
                Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC
        }
    }
}
