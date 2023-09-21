package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * A container class for constants used in various places in the project.
 */
public final class Constants{
    public static final double STICK_DEADBAND = 0.05;
    public static final boolean INCLUDE_DEBUG_TABS = true;

    public static final class SwerveConstants {

        public static final boolean UPDATE_SWERVE_FROM_SHUFFLEBOARD = false;

        public static final boolean INVERT_GYRO = false; // TODO Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = COTSFalconSwerveConstants
                            .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3); // TODO ROBOT SPECIFIC

        /* Drivetrain Constants */ // TODO ROBOT SPECIFIC
        public static final double TRACK_WIDTH = 0.50165; // Distance from left wheels to right wheels/vice versa
        public static final double WHEEL_BASE = 0.57531; // Distance from front wheels to back wheels/vice versa
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertType ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final InvertType DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * A small open loop ramp (0.25) can help with tread wear, tipping, etc.
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_KF = CHOSEN_MODULE.angleKF;
        public static final ScreamPIDConstants ANGLE_CONSTANTS = new ScreamPIDConstants(ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF);

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.05; // TODO ROBOT SPECIFIC
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;
        public static final ScreamPIDConstants DRIVE_CONSTANTS = new ScreamPIDConstants(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF);

        public static final double DRIVE_KS = (0.32 / 12); // TODO ROBOT SPECIFIC
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);

        /* PathPlanner PIDConstants */
        public static final PIDController PATH_TRANSLATION_CONTROLLER = new PIDController(0.5, 0.0, 0.0); // TODO ROBOT SPECIFIC
        public static final PIDController PATH_ROTATION_CONTROLLER = new PIDController(0.5, 0.0, 0.0);

        /* Swerve Profiling Values */

        /* Meters per Second */
        public static final double MAX_SPEED = 5.7349; // TODO ROBOT SPECIFIC

        /* Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 8.0; // TODO ROBOT SPECIFIC

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast; // TODO CHANGE TO BRAKE AFTER MEASURING OFFSETS
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Front Left */
        public static final SwerveModuleConstants MODULE_0 = new SwerveModuleConstants(
            23, 
            24, 
            8, 
            Rotation2d.fromDegrees(3.33984375)); // TODO ROBOT SPECIFIC

        /* Front Right */
        public static final SwerveModuleConstants MODULE_1 = new SwerveModuleConstants(
            13, 
            14, 
            3, 
            Rotation2d.fromDegrees(-52.9101562)); // TODO ROBOT SPECIFIC

        /* Back Left */
        public static final SwerveModuleConstants MODULE_2 = new SwerveModuleConstants(
            19, 
            20, 
            6, 
            Rotation2d.fromDegrees(-76.90429687500001)); // TODO ROBOT SPECIFIC

        /* Back Right */
        public static final SwerveModuleConstants MODULE_3 = new SwerveModuleConstants(
            11, 
            12, 
            2, 
            Rotation2d.fromDegrees(-69.34570312500003)); // TODO ROBOT SPECIFIC

        /** 
         * Use this if you have multiple sets of modules.
         * In our case, this is useful for hotswapping modules.
         * Simply set each location's constants to their corresponding module.
         */
        public static final SwerveModuleConstants FRONT_LEFT_MODULE = MODULE_0;
        public static final SwerveModuleConstants FRONT_RIGHT_MODULE = MODULE_1;
        public static final SwerveModuleConstants BACK_LEFT_MODULE = MODULE_2;
        public static final SwerveModuleConstants BACK_RIGHT_MODULE = MODULE_3;

    }

    public static final class Ports {
        /* CANivore */
        public static final String CANIVORE_BUS_NAME = "canivore"; // TODO DELETE IF ROBOT DOES NOT USE A CANivore

        /* Sensor ports */
        public static final int PIGEON_ID = 0;
    }

}
