package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants.Modules;
import frc.robot.auto.AutoEvents;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
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
 * 
 * This class provides methods for high-level control of the swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private Pigeon2 m_gyro;
    private SwerveModule[] m_swerveModules;
    private SwerveDriveOdometry m_swerveOdometry;

    /**
     * Constructs a new instance of the Swerve class.
     * 
     * Initializes the gyro, swerve modules, and odometry.
     */
    public Swerve() {
        m_gyro = new Pigeon2(Ports.PIGEON_ID, Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
        configGyro();
        
        /**
         * Initializes an array of SwerveModule objects with their respective names, IDs, and constants.
         * This array represents the robot's four swerve modules.
         */
        m_swerveModules = new SwerveModule[] {
                new SwerveModule("FL", 0, Modules.FRONT_LEFT.getAssociated()),
                new SwerveModule("FR", 1, Modules.FRONT_RIGHT.getAssociated()),
                new SwerveModule("BL", 2, Modules.BACK_LEFT.getAssociated()),
                new SwerveModule("BR", 3, Modules.BACK_RIGHT.getAssociated())
        };
        
        /**
         * Configures the odometry, which requires the kinematics, gyro reading, and module positions.
         * It uses these values to estimate the robot's position on the field.
         */
        m_swerveOdometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getYaw(),
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
     * @param angularVel The desired angular velocity in rads/sec.
     * @param fieldRelative Whether the movement should be field-relative or robot-relative.
     * @param isOpenLoop Whether the driving should be open loop (Tele-Op driving) or closed loop (Autonomous driving).
     */
    public void drive(Translation2d translation, double angularVel, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        angularVel,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                angularVel));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    /**
     * Drives the swerve drive system based on the given chassis speeds.
     * Used as an input for followTrajectoryCommand.
     *
     * @param chassisSpeeds The desired chassis speeds to drive.
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], false);
        }
    }

    /** 
     * Calculates the hold value based on the provided measurement and setpoint. 
     *  
     * @param measurement The current measurement value. 
     * @param setpoint The desired setpoint value. 
     * @return The calculated value from the hold controller.
    */
    public double calculateHold(double measurement, double setpoint){
        return SwerveConstants.SWERVE_HOLD_CONTROLLER.calculate(measurement, setpoint);
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
     * Resets the pose reported by the odometry to the initial pose of the specified trajectory.
     * For use in auto routines.
     *
     * @param trajectory The trajectory to get the inital pose from.
     */
    public void resetPose(PathPlannerTrajectory trajectory) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), trajectory.getInitialHolonomicPose() );
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
     * If {@code}invertGyro{@code} is set to true, the yaw rotation is inverted.
     *
     * @return The yaw rotation as a Rotation2d.
     */
    public Rotation2d getYaw() {
        return (SwerveConstants.GYRO_INVERT) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw().getValue())
                : Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
    }

    /**
     * Configures the gyro. Resets it to factory default settings and zeroes it.
     */
    public void configGyro(){
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
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
     * @return The command that follows the trajectory and triggers associated events along it.
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean mirrorWithAlliance) {
        FollowPathWithEvents path = new FollowPathWithEvents(
            new PPSwerveControllerCommand(
                trajectory, 
                this::getPose, // Pose supplier
                SwerveConstants.SWERVE_KINEMATICS, // SwerveDriveKinematics
                SwerveConstants.PATH_TRANSLATION_CONTROLLER, // X controller
                SwerveConstants.PATH_TRANSLATION_CONTROLLER, // Y controller 
                SwerveConstants.PATH_ROTATION_CONTROLLER, // Rotation controller
                this::setModuleStates, // Module states consumer
                mirrorWithAlliance, // If the path should be mirrored depending on alliance color
                this // Requires this drive subsystem
            ), 
            trajectory.getMarkers(), 
            AutoEvents.getEvents());
            return path;
    }
}