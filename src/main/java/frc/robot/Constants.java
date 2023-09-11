package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * A container class for constants used in various places in the project.
 */
public final class Constants {
    public static final double stickDeadband = 0.05;
    public static final boolean includeDebugTabs = true;

    public static final class SwerveConstants {

        public static final boolean updateSwerveFromShuffleboard = false;

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                            .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3); // TODO ROBOT SPECIFIC

        /* Drivetrain Constants */
        public static final double trackWidth = 0.0; // TODO ROBOT SPECIFIC
        public static final double wheelBase = 0.0; // TODO ROBOT SPECIFIC
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * A small open loop ramp (0.25) can help with tread wear, tipping, etc.
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0; // TODO ROBOT SPECIFIC
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        public static final double driveKS = (0.32 / 12); // TODO ROBOT SPECIFIC
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* PathPlanner PIDConstants */
        public static final PIDConstants pathTranslationConstants = new PIDConstants(0.0, 0.0, 0.0); // TODO ROBOT SPECIFIC
        public static final PIDConstants pathRotationConstants = new PIDConstants(0.0, 0.0, 0.0);

        /* Swerve Profiling Values */

        /* Meters per Second */
        public static final double maxSpeed = 0.0; // TODO ROBOT SPECIFIC

        /* Radians per Second */
        public static final double maxAngularVelocity = 0.0; // TODO ROBOT SPECIFIC

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Front Left */
        public static final SwerveModuleConstants Module0 = new SwerveModuleConstants(
            23, 
            24, 
            8, 
            Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC

        /* Front Right */
        public static final SwerveModuleConstants Module1 = new SwerveModuleConstants(
            13, 
            14, 
            3, 
            Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC

        /* Back Left */
        public static final SwerveModuleConstants Module2 = new SwerveModuleConstants(
            19, 
            20, 
            6, 
            Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC

        /* Back Right */
        public static final SwerveModuleConstants Module3 = new SwerveModuleConstants(
            11, 
            12, 
            2, 
            Rotation2d.fromDegrees(0.0)); // TODO ROBOT SPECIFIC
    }

    public static final class Ports {
        /* CANivore */
        public static final String canivoreBusName = "canivore"; // TODO DELETE IF ROBOT DOES NOT USE A CANivore

        /* Limelight */
        public static final String limelightName = "limelight";

        /* Electrical component ports */
        public static final int pdhID = 0;

        /* Sensor ports */
        public static final int pigeonID = 0;
    }

}
