package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveModule;
import frc.lib.pid.ScreamPIDConstants;
import frc.lib.util.ScreamUtil;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
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
    private ChassisSpeeds m_currentSpeeds;

    /**
     * Constructs a new instance of the Swerve class.
     * 
     * Initializes the gyro, swerve modules, and odometry.
     */
    public Swerve() {
        m_gyro = new Pigeon2(Ports.PIGEON_ID, Ports.CAN_BUS_NAME);
        configGyro();
        
        /**
         * Initializes an array of SwerveModule objects with their respective names, IDs, and constants.
         * This array represents the robot's four swerve modules.
         */
        m_swerveModules = new SwerveModule[] {
                new SwerveModule(SwerveConstants.FRONT_LEFT),
                new SwerveModule(SwerveConstants.FRONT_RIGHT),
                new SwerveModule(SwerveConstants.BACK_LEFT),
                new SwerveModule(SwerveConstants.BACK_RIGHT)
        };
        
        /**
         * Configures the odometry, which requires the kinematics, gyro reading, and module positions.
         * It uses these values to estimate the robot's position on the field.
         */
        m_swerveOdometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions(), new Pose2d());

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getRobotCentricSpeeds,
            this::setChassisSpeeds,
            SwerveConstants.PATH_FOLLOWER_CONFIG,
            this
        );
    }

    /**
     * Resets the yaw of the gyro to zero.
     */
    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * Returns a new ChassisSpeeds based on the given inputs.
     *
     * @param translation A Translation2d representing the desired movement in x and y directions.
     * @param angularVel The desired angular velocity in rads/sec.
     * @param fieldRelative Whether the speeds should be field-relative or robot-relative.
     * @return The calculated ChassisSpeeds.
     */
    public ChassisSpeeds robotSpeeds(Translation2d translation, double angularVel, boolean fieldCentric){
        ChassisSpeeds speeds = fieldCentric 
                      ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), angularVel, getYaw())
                      : new ChassisSpeeds(translation.getX(), translation.getY(), angularVel);
        return ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME_SEC);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop){
        m_currentSpeeds = chassisSpeeds;
        SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        setChassisSpeeds(chassisSpeeds, false);
    }

    /**
     * Drives the swerve drive system based on the given chassis speeds.
     * Used as an input for FollowTrajectoryCommand.
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
    public double calculateHeadingCorrection(double measurement, double setpoint){
        return SwerveConstants.HEADING_CORRECT_CONTROLLER.calculate(measurement, setpoint);
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
    public void resetPose(PathPlannerPath trajectory) {
        Translation2d location = trajectory.getAllPathPoints().get(0).position;
        Rotation2d angle = trajectory.getAllPathPoints().get(0).holonomicRotation;
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), new Pose2d(location, angle));
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
            states[mod.getModuleNumber()] = mod.getState(true);
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
            positions[mod.getModuleNumber()] = mod.getPosition(true);
        }
        return positions;
    }

    public ChassisSpeeds getRobotCentricSpeeds(){
        return m_currentSpeeds;
    }

    /**
     * Returns the yaw rotation in degrees.
     * If {@code}invertGyro{@code} is set to true, the yaw rotation is inverted.
     *
     * @return The yaw rotation as a Rotation2d.
     */
    public Rotation2d getYaw() {
        return (SwerveConstants.GYRO_INVERT) ? m_gyro.getRotation2d().minus(Rotation2d.fromDegrees(360))
                : m_gyro.getRotation2d();
    }

    public Pigeon2 getGyro() {
        return m_gyro;
    }

    /**
     * Configures the gyro. Resets it to factory default settings and zeroes it.
     */
    public void configGyro() {
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.getYaw().setUpdateFrequency(50);
        m_gyro.optimizeBusUtilization();
        zeroGyro();
    }

    public void configDrivePID(ScreamPIDConstants constants){
        for (SwerveModule mod : m_swerveModules) {
            mod.configDriveMotorPID(constants);
        }
    }

    /**
     * Called periodically through SubsystemBase
     */
    @Override
    public void periodic() {
        m_swerveOdometry.update(getYaw(), getModulePositions()); /* Updates the pose estimator with the current angle and module positions */
    }
}