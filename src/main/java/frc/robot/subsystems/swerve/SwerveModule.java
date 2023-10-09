package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.AngleConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants.SwerveModuleConstants;

/**
 * A swerve module, which consists of an angle motor, a drive motor, and an angle encoder.
 * 
 * This class provides methods to get and set all parts of the module, including the speed and angle.
 */
public class SwerveModule {
    private int m_modNumber;
    private String m_modLocation;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANcoder m_angleEncoder;

    private DutyCycleOut m_driveCycle = new DutyCycleOut(0);
    private VelocityVoltage m_driveVelVoltage = new VelocityVoltage(0);
    private PositionVoltage m_anglePosVoltage = new PositionVoltage(0);

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA);

    /**
     * Constructs a SwerveModule object with the given location, module number, and module constants.
     *
     * @param modLocation The location of the SwerveModule.
     * @param moduleNumber The module number of the SwerveModule.
     * @param moduleConstants The constants specific to this SwerveModule.
     */
    public SwerveModule(String modLocation, int modNumber, SwerveModuleConstants moduleConstants) {
        m_modLocation = modLocation;
        m_modNumber = modNumber;
        m_angleOffset = moduleConstants.angleOffset();

        /* Angle Encoder Config */
        m_angleEncoder = new CANcoder(moduleConstants.encoderID(), Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
        configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID(), Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID(), Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
        configDriveMotor();

        m_lastAngle = getState().angle;
    }

    /**
     * Returns the location associated with this module.
     *
     * @return The module location.
     */
    public String getLocation() {
        return m_modLocation;
    }

    /**
     * Returns the module number associated with this module.
     *
     * @return The module number.
     */
    public int getModuleNumber() {
        return m_modNumber;
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
            m_driveCycle.Output = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            m_driveMotor.setControl(m_driveCycle);
        } else {
            m_driveVelVoltage.Velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    SwerveConstants.WHEEL_CIRCUMFERENCE, DriveConstants.GEAR_RATIO);
            m_driveVelVoltage.FeedForward = m_feedforward.calculate(desiredState.speedMetersPerSecond);
            m_driveMotor.setControl(m_driveVelVoltage);
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
        m_anglePosVoltage.Position = Conversions.degreesToFalcon(angle.getDegrees(), AngleConstants.GEAR_RATIO);
        m_angleMotor.setControl(m_anglePosVoltage);
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
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getPosition().refresh().getValue(), AngleConstants.GEAR_RATIO));
    }

    /**
     * Retrieves the current absolute rotation of the CANcoder.
     *
     * @return The current absolute rotation of the CANcoder sensor as a Rotation2d.
     */
    public Rotation2d getEncoder() {
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
    }

    /**
     * Resets the angle motor to the absolute position provided by the CANcoder.
     * 
     * Delays for 500 ms to prevent problems with getting the CANcoder angle before it is initialized.
     */
    public void resetToAbsolute() {
        Rotation2d position = Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().waitForUpdate(0.5).getValue());
        m_angleMotor.setPosition(Conversions.degreesToFalcon(position.minus(m_angleOffset).getDegrees(), AngleConstants.GEAR_RATIO));
    }

    /**
     * Configures the angle encoder.
     * 
     * Configures the encoder with the specified configuration.
     */
    private void configAngleEncoder() {
        DeviceConfig.configureSwerveEncoder(m_modLocation, m_angleEncoder, DeviceConfig.swerveEncoderConfig());
    }

    /**
     * Configures the angle motor.
     * 
     * Configures the motor with the specified configuration.
     */
    private void configAngleMotor() {
        DeviceConfig.configureTalonFX(m_modLocation, m_angleMotor, DeviceConfig.angleFXConfig());
        resetToAbsolute();
    }

    private void configAngleMotorPID(ScreamPIDConstants constants) {
        m_angleMotor.getConfigurator().apply(constants.slot0Configs());
    }

    /**
     * Configures the drive motor.
     * 
     * Configures the motor with the specified configuration.
     */
    private void configDriveMotor() {
        DeviceConfig.configureTalonFX(m_modLocation, m_driveMotor, DeviceConfig.driveFXConfig());
    }

    /**
     * Retrieves the current state of the swerve module.
     *
     * @return The state of the swerve module, including the velocity (m/s) and angle.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(m_driveMotor.getVelocity().refresh().getValue(), SwerveConstants.WHEEL_CIRCUMFERENCE, DriveConstants.GEAR_RATIO), getAngle());
    }

    /**
     * Retrieves the current position of the swerve module.
     *
     * @return The position of the swerve module, consisting of the distance traveled in meters and the angle.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(m_driveMotor.getPosition().refresh().getValue(), SwerveConstants.WHEEL_CIRCUMFERENCE, DriveConstants.GEAR_RATIO), getAngle());
    }
}