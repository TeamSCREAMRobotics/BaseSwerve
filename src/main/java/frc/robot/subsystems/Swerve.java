package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.AutoEvents;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.HashMap;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
     * Initializes the gyro, swerve modules, and odometry.
     */
    public Swerve() {
        m_gyro = new Pigeon2(Ports.pigeonID, Ports.canivoreBusName); //TODO delete canivoreBusName if the robot is not using a CANivore
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
         * Configures the odometry, which uses the kinematics, gyro reading, and module positions.
         * It uses these values to estimate the robot's position on the field.
         */
        m_swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(),
                getModulePositions(), new Pose2d());
    }

    /**
     * Resets the yaw of the gyro to zero.
     */
    public void zeroGyro() {
        m_gyro.setYaw(0);
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
     * @param chassisSpeeds The desired chassis speeds to drive.
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        //SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], false);
        }
    }

    /**
     * Resets the pose reported by the odometry to the specified pose.
     *
     * @param pose The new pose to set.
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
     * If <STRONG>invertGyro</STRONG> is set to true, the yaw rotation is inverted.
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
     * Will automatically trigger events associated with that trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @param mirrorWithAlliance If the trajectory should be flipped according to alliance color.
     * @return The full path, including the events triggered along it.
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean mirrorWithAlliance) {
        HashMap<String, Command> autoEvents = AutoEvents.getEvents();

        FollowPathWithEvents path = new FollowPathWithEvents(
            new PPSwerveControllerCommand(
                trajectory, 
                this::getPose, // Pose supplier
                SwerveConstants.swerveKinematics, // SwerveDriveKinematics
                SwerveConstants.pathTranslationController, // X controller
                SwerveConstants.pathTranslationController, // Y controller 
                SwerveConstants.pathRotationController, // Rotation controller
                this::setModuleStates, // Module states consumer
                mirrorWithAlliance, // If the path should be mirrored depending on alliance color
                this // Requires this drive subsystem
            ), 
            trajectory.getMarkers(), 
            autoEvents);
            return path;
    }

    /**
     * Generates a command that follows the given trajectory.
     * Will automatically trigger events associated with that trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @return The full path, including the events triggered along it.
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory) {
        HashMap<String, Command> autoEvents = AutoEvents.getEvents();

        FollowPathWithEvents path = new FollowPathWithEvents(
            new PPSwerveControllerCommand(
                trajectory, 
                this::getPose, // Pose supplier
                SwerveConstants.swerveKinematics, // SwerveDriveKinematics
                SwerveConstants.pathTranslationController, // X controller
                SwerveConstants.pathTranslationController, // Y controller 
                SwerveConstants.pathRotationController, // Rotation controller
                this::setModuleStates, // Module states consumer
                true, // If the path should be mirrored depending on alliance color
                this // Requires this drive subsystem
            ), 
            trajectory.getMarkers(), 
            autoEvents);
            return path;
    }
}