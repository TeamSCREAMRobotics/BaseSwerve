package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.deviceConfiguration.DeviceConfig;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;

/**
 * A swerve module, which consists of an angle motor, a drive motor, and an angle encoder.
 * 
 * This class provides methods to get and set all parts of the module, including the speed and angle.
 */
public class SwerveModule {
    private int m_moduleNumber;
    private String m_location;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANcoder m_angleEncoder;

    private DutyCycleOut driveCycle = new DutyCycleOut(0);
    private VelocityVoltage driveVelVoltage = new VelocityVoltage(0);
    private PositionVoltage anglePosVoltage = new PositionVoltage(0);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV,
            SwerveConstants.DRIVE_KA);

    /**
     * Constructs a SwerveModule object with the given location, module number, and module constants.
     *
     * @param location The location of the SwerveModule.
     * @param moduleNumber The module number of the SwerveModule.
     * @param moduleConstants The constants specific to this SwerveModule.
     */
    public SwerveModule(String location, int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.m_moduleNumber = moduleNumber;
        this.m_location = location;
        this.m_angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        m_angleEncoder = new CANcoder(moduleConstants.CANcoderID, Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
        configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID, Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
        configDriveMotor();

        m_lastAngle = getState().angle;
    }

    /**
     * Returns the location associated with this module.
     *
     * @return The location.
     */
    public String getLocation() {
        return m_location;
    }

    /**
     * Returns the module number.
     *
     * @return The module number.
     */
    public int getModuleNumber() {
        return m_moduleNumber;
    }

    public void setAngleNeutralMode(NeutralModeValue mode){
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.NeutralMode = mode;
        m_angleMotor.getConfigurator().refresh(configs);
    }

    public void setDriveNeutralMode(NeutralModeValue mode){
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.NeutralMode = mode;
        m_driveMotor.getConfigurator().refresh(configs);
    }

    /**
     * Sets the desired state of the Swerve module, including the angle and the speed
     *
     * @param desiredState The desired state of the Swerve module.
     * @param isOpenLoop   A boolean indicating whether the module is in open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */
    public void set(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speed of the swerve module based on the desired state's speed and whether it is in open loop or closed loop control.
     *
     * @param desiredState The desired state of the swerve module.
     * @param isOpenLoop   A boolean indicating whether the module is in open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveCycle.Output = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            m_driveMotor.setControl(driveCycle);
        } else {
            driveVelVoltage.Velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO);
            driveVelVoltage.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
            m_driveMotor.setControl(driveVelVoltage);
        }
    }

    /**
     * Sets the angle of the swerve module based on the desired state's angle.
     *
     * @param desiredState The desired state of the swerve module.
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED * 0.01))
                ? m_lastAngle
                : desiredState.angle; /* Prevent rotating module if speed is less then 1%. Prevents jittering when not moving. */
        anglePosVoltage.Position = Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO);
        m_angleMotor.setControl(anglePosVoltage);
        m_lastAngle = angle;
    }

    /**
     * Optimizes the movement of the module to travel the least distance to get to the desired angle.
     *
     * @param desiredState The desired state of the swerve module.
     * @param currentAngle The current angle of the swerve module as a Rotation2d.
     * @return The optimized swerve module state.
     */
    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
		double deltaDegrees = desiredState.angle.minus(currentAngle).getDegrees();
		boolean invert = Math.abs(deltaDegrees) > 90;
		if(invert) deltaDegrees -= Math.signum(deltaDegrees)*180;
		return new SwerveModuleState(
			(invert ? -1 : 1) * desiredState.speedMetersPerSecond,
			Rotation2d.fromDegrees(currentAngle.getDegrees() + deltaDegrees)
		);
	}

    /**
     * Retrieves the current agnle of the angle motor.
     *
     * @return The angle of the rotation motor as a Rotation2d.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(m_angleMotor.getPosition().getValue(), SwerveConstants.ANGLE_GEAR_RATIO));
    }

    /**
     * Retrieves the current absolute rotation of the CANcoder.
     *
     * @return The current absolute rotation of the CANcoder sensor, adjusted by the angle offset, as a Rotation2d.
     */
    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition().getValue()).minus(m_angleOffset);
    }

    /**
     * Resets the angle motor to the absolute position provided by the CANcoder.
     * 
     * Delays for 1 second to prevent problems with getting the CANcoder angle before it is initialized.
     */
    public void resetToAbsolute() {
        double position = Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().waitForUpdate(0.5).getValue()).getDegrees();
        m_angleMotor.setRotorPosition(Conversions.degreesToFalcon(position - m_angleOffset.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO));
    }

    /**
     * Configures the angle encoder for the swerve module.
     * 
     * Resets to factory default settings and configures the invert mode.
     */
    private void configAngleEncoder() {
        DeviceConfig.configSwerveCANcoder(
            m_angleEncoder, 
            m_location,
            SwerveConstants.CANCODER_INVERT);
    }

    /**
     * Configures the angle motor for swerve drive.
     * 
     * Configures basic settings, the inversion and neutral modes, and resets it to absolute.
     */
    private void configAngleMotor() {
        DeviceConfig.configSwerveAngleFX(
            m_angleMotor, 
            m_location,
            SwerveConstants.ANGLE_MOTOR_INVERT, 
            SwerveConstants.ANGLE_NEUTRAL_MODE, 
            SwerveConstants.ANGLE_CONSTANTS);
        resetToAbsolute();
    }

    /**
     * Configures the drive motor with the specified settings.
     * 
     * Configures basic settings, the inversion and neutral modes, and resets the position to zero.
     */
    private void configDriveMotor() {
        DeviceConfig.configSwerveDriveFX(
            m_driveMotor, 
            m_location,
            SwerveConstants.DRIVE_MOTOR_INVERT, 
            SwerveConstants.DRIVE_NEUTRAL_MODE, 
            SwerveConstants.DRIVE_CONSTANTS);
    }

    /**
     * Retrieves the current state of the swerve module.
     *
     * @return The state of the swerve module, including the velocity (m/s) and angle.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(m_driveMotor.getVelocity().getValue(), SwerveConstants.WHEEL_CIRCUMFERENCE,
                        SwerveConstants.DRIVE_GEAR_RATIO),
                getAngle());
    }

    /**
     * Retrieves the current position of the swerve module.
     *
     * @return The position of the swerve module, consisting of the distance traveled in meters and the angle.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(m_driveMotor.getPosition().getValue(), SwerveConstants.WHEEL_CIRCUMFERENCE,
                        SwerveConstants.DRIVE_GEAR_RATIO),
                getAngle());
    }
}