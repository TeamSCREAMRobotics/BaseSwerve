package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DeviceConfig;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveModule;

/**
 * A swerve drive subsystem.
 * 
 * This class provides methods for high-level control of the swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private Pigeon2 m_gyro;
    private SwerveModule[] m_swerveModules;
    private SwerveDriveOdometry m_odometry;
    private ChassisSpeeds m_currentSpeeds = new ChassisSpeeds();

    /**
     * Constructs a new instance of the Swerve class.
     * 
     * Initializes the gyro, swerve modules, odometry, and auto builder.
     */
    public Swerve() {
        m_gyro = new Pigeon2(Ports.PIGEON_ID, Ports.CAN_BUS_NAME);
        configGyro();
        
        /**
         * Initializes an array of SwerveModule objects with their respective names, IDs, and constants.
         * This array represents the robot's four swerve modules.
         * If there are multiple sets of modules, swap out the constants for the module in use.
         */
        m_swerveModules = new SwerveModule[] {
                new SwerveModule(0, ModuleConstants.MODULE_0), // Front Left
                new SwerveModule(1, ModuleConstants.MODULE_1), // Front Right
                new SwerveModule(2, ModuleConstants.MODULE_2), // Back Left
                new SwerveModule(3, ModuleConstants.MODULE_3)  // Back Right
        };
        
        /**
         * Configures the odometry, which requires the kinematics, gyro reading, and module positions.
         * It uses these values to estimate the robot's position on the field.
         */
        m_odometry = new SwerveDriveOdometry(SwerveConstants.KINEMATICS, getYaw(), getModulePositions(), new Pose2d());

        /**
         * Configures the AutoBuilder for holonomic mode.
         * The AutoBuilder uses methods from this class to follow paths.
         */
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
     * @param translation A Translation2d representing the desired movement (m/s) in the x and y directions.
     * @param angularVel The desired angular velocity (rad/s)
     * @param fieldRelative Whether the speeds should be field or robot centric.
     * @return The calculated ChassisSpeeds.
     */
    public ChassisSpeeds robotSpeeds(Translation2d translation, double angularVel, boolean fieldCentric){
        ChassisSpeeds speeds = fieldCentric 
                      ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), angularVel, getYaw())
                      : new ChassisSpeeds(translation.getX(), translation.getY(), angularVel);

        return ChassisSpeeds.discretize(speeds, Constants.LOOP_TIME_SEC);
    }

    /**
     * Set the ChassisSpeeds to drive the robot. Use predefined methods such as {@code}robotSpeeds{@code} or create a new ChassisSpeeds object.
     * 
     * @param chassisSpeeds The ChassisSpeeds to generate states for.
     * @param isOpenLoop Whether the ChassisSpeeds is open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
        m_currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(swerveModuleStates);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    /**
     * Set the ChassisSpeeds to drive the robot. Defaults to closed loop.<p>
     * Used by AutoBuilder.
     * 
     * @param chassisSpeeds The ChassisSpeeds to generate states for.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        setChassisSpeeds(chassisSpeeds, false);
    }

    /**
     * Sets the neutral mode of the motors.<p>
     * Use {@code}NeutralModeValue.Brake{@code} or {@code}NeutralModeValue.Coast{@code}
     * 
     * @param driveMode The NeutralModeValue to set the drive motor to.
     * @param steerMode The NeutralModeValue to set the drive motor to.
     */
    public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode){
        for (SwerveModule mod : m_swerveModules) {
            mod.setDriveNeutralMode(driveMode);
            mod.setSteerNeutralMode(steerMode);
        }
    }

    /** 
     * Calculates the hold value based on the provided current and last angles.
     *  
     * @param currentAngle The current angle measurement.
     * @param lastAngle The desired angle to calculate towards.
     * @return The calculated value from the heading controller.
    */
    public double calculateHeadingCorrection(double currentAngle, double lastAngle){
        return SwerveConstants.HEADING_CONSTANTS.toPIDController().calculate(currentAngle, lastAngle);
    }

    private Timer coastTimer = new Timer();
    /**
     * Checks if the robot is disabled then sets the motors to coast after the specified amount of seconds.
     * 
     * @param elapsedSec Amount of seconds to wait after disable.
     */
    public void coastAfterDisable(double elapsedSec){
        if(DriverStation.isEnabled()){
            setNeutralModes(NeutralModeValue.Brake, NeutralModeValue.Brake);
            coastTimer.reset();
        } else {
            coastTimer.start();
            if(coastTimer.hasElapsed(elapsedSec)){
                setNeutralModes(NeutralModeValue.Coast, NeutralModeValue.Coast);
            }
        }
    }
    

    /**
     * Resets the pose reported by the odometry to the specified pose.
     *
     * @param pose The new pose to set.
     */
    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Retrieves the estimated pose of the odometry.
     *
     * @return The current pose of the odometry.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns an array composed of the swerve modules in the system.
     *
     * @return The array of SwerveModule objects.
     */
    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    /**
     * Returns an array composed of the state of each module in the system.
     *
     * @return The array of SwerveModuleState objects.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveModules) {
            states[mod.getModuleNumber()] = mod.getState(true);
        }
        return states;
    }

    /**
     * Returns an array composed of the positions of each module in the system.
     *
     * @return The array of SwerveModulePosition objects.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveModules) {
            positions[mod.getModuleNumber()] = mod.getPosition(true);
        }
        return positions;
    }

    /**
     * Returns the current robot-centric ChassisSpeeds.<p>
     * Used by AutoBuilder.
     * 
     * @return The current robot-centric ChassisSpeeds.
     */
    public ChassisSpeeds getRobotCentricSpeeds(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(m_currentSpeeds, getYaw());
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

    /**
     * Returns the gyro object.<p>
     * Used by SwerveTab to display to Shuffleboard.
     * 
     * @return The gyro object.
     */
    public Pigeon2 getGyro() {
        return m_gyro;
    }

    /**
     * Configures the gyro. Resets it to factory default settings and zeroes it.
     */
    public void configGyro() {
        DeviceConfig.configurePigeon2("Swerve Pigeon", m_gyro, DeviceConfig.swervePigeonConfig(), Constants.LOOP_TIME_HZ);
    }

   /**
     * Configures all module drive motors with the given constants.
     * 
     * @param constants ScreamPIDConstants to be applied.
     */
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
        m_odometry.update(getYaw(), getModulePositions()); /* Updates the odometry with the current angle and module positions */
        coastAfterDisable(5);
    }
}