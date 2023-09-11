package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.HashMap;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A swerve drive subsystem.
 * <p>
 * This class provides methods for high-level control of the swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry m_swerveOdometry;
    private SwerveModule[] m_swerveModules;
    private Pigeon2 m_gyro;

    /**
     * Constructs a new instance of the Swerve class.
     * <p>
     * Initializes the gyro, swerve modules, and pose estimator.
     */
    public Swerve() {
        m_gyro = new Pigeon2(Ports.pigeonID, Ports.canivoreBusName); //TODO delete Ports.canivoreBusName if the robot is not using a CANivore
        configGyro();
        
        /**
         * Initializes an array of SwerveModule objects with their respective names, IDs, and constants.
         */
        m_swerveModules = new SwerveModule[] {
                new SwerveModule("FL", 0, SwerveConstants.Module0),
                new SwerveModule("FR", 1, SwerveConstants.Module1),
                new SwerveModule("BL", 2, SwerveConstants.Module2),
                new SwerveModule("BR", 3, SwerveConstants.Module3)
        };
        
        /**
         * Configures the pose estimator, which uses the kinematics, gyro reading, and module positions.
         * It uses these values to estimate the robot's position on the field.
         */
        m_swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(),
                getModulePositions(), new Pose2d());

        zeroPose();
    }

    /**
     * Resets the yaw of the gyro to zero.
     */
    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * Resets the pose of the pose estimator
     */
    public void zeroPose() {
        m_swerveOdometry.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), new Pose2d());
    }

    /**
     * Drives the swerve drive system based on the given translation and rotation inputs.
     * Uses inputs for field relative control and if the control is open-loop.
     *
     * @param translation A Translation2d representing the desired movement in x and y directions.
     * @param rotation The rotation value representing the desired rotation speed.
     * @param fieldRelative Whether the movement should be field-relative or robot-relative.
     * @param isOpenLoop Whether the drive should be open loop (Tele-Op driving) or closed loop (Autonomous driving).
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    /**
     * Drives the swerve drive system based on the given chassis speeds.
     * Used as an input for SwerveAutoBuilder.
     *
     * @param chassisSpeeds The desired chassis speeds to drive the swerve drive system.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], false);
        }
    }

    /**
     * Resets the pose reported by the odometry to the specified pose.
     *
     * @param pose The new pose to set for the swerve drive system.
     */
    public void resetPose(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Retrieves the estimated pose of the odometry.
     *
     * @return The current pose of the odometry.
     */
    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    /**
     * Returns an array of SwerveModule objects representing the swerve modules in the system.
     *
     * @return An array of SwerveModule objects.
     */
    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    /**
     * Retrieves the current state of all swerve modules.
     *
     * @return An array of SwerveModuleState objects representing the state of each swerve module.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    /**
     * Retrieves the positions of all swerve modules.
     *
     * @return An array of SwerveModulePosition objects representing the positions of each swerve module.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveModules) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Returns the yaw rotation in degrees.
     * If <STRONG>SwerveConstants.invertGyro</STRONG> is set to true, the yaw rotation is inverted.
     *
     * @return The yaw rotation in degrees.
     */
    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
                : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    /**
     * Configures the gyro by resetting it to factory default settings and zeroing it.
     */
    public void configGyro(){
        m_gyro.configFactoryDefault();
        zeroGyro();
    }

    /**
     * Called periodically through SubsystemBase
     */
    @Override
    public void periodic() {
        m_swerveOdometry.update(getYaw(), getModulePositions()); /* Updates the pose estimator with the current angle and module positions */
    }

    /**
     * Generates a command that follows the given trajectory.
     * Use an event map to trigger given events along the trajectory.
     *
     * @param traj The trajectory to follow.
     * @param eventMap A map of events to be triggered during the trajectory.
     * @param isFirstPath Indicates if this is the first path in the sequence.
     * @param useAllianceColor Indicates if the alliance color should be used.
     * @return The command to follow the path, including the triggered.
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, HashMap<String, Command> eventMap,
            boolean isFirstPath, boolean useAllianceColor) {
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                this::getPose,
                this::resetPose,
                SwerveConstants.pathTranslationConstants,
                SwerveConstants.pathRotationConstants,
                this::drive,
                eventMap,
                useAllianceColor,
                this);

        return autoBuilder.fullAuto(traj);
    }
}