package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import frc.lib.config.DeviceConfig;
import frc.lib.math.Conversions;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants.Module;

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

    private TalonFX m_steerMotor;
    private TalonFX m_driveMotor;
    private CANcoder m_angleEncoder;

    private StatusSignal<Double> m_drivePosition;
    //private StatusSignal<Double> m_driveVelocity;
    private StatusSignal<Double> m_steerPosition;
    private StatusSignal<Double> m_steerVelocity;
    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    private DutyCycleOut m_driveCycle = new DutyCycleOut(0);
    private VelocityVoltage m_driveVoltage = new VelocityVoltage(0);
    private PositionVoltage m_steerVoltage = new PositionVoltage(0);

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA);

    /**
     * Constructs a SwerveModule object with the given location, module number, and module constants.
     *
     * @param module The enum to get the location, number, and constants from.
     * Set each module's constants in ModuleConstants.
     */
    public SwerveModule(Module module) {
        m_modNumber = module.number();
        m_modLocation = module.toString();
        m_angleOffset = module.constants().angleOffset();

        /* Angle Encoder Config */
        m_angleEncoder = new CANcoder(module.constants().encoderID(), Ports.CAN_BUS_NAME);
        configAngleEncoder();

        /* Steer Motor Config */
        m_steerMotor = new TalonFX(module.constants().steerMotorID(), Ports.CAN_BUS_NAME);
        configSteerMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(module.constants().driveMotorID(), Ports.CAN_BUS_NAME);
        configDriveMotor();

        m_lastAngle = getState(true).angle;

        m_drivePosition = m_driveMotor.getPosition();
        //m_driveVelocity = m_driveMotor.getVelocity();
        m_steerPosition = m_steerMotor.getPosition();
        m_steerVelocity = m_steerMotor.getVelocity();
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

    public void setSteerNeutralMode(NeutralModeValue mode){
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.NeutralMode = mode;
        m_steerMotor.getConfigurator().refresh(configs);
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
        desiredState = SwerveModuleState.optimize(desiredState, getState(true).angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speed of the swerve module based on the desired state's speed and whether it is in open loop or closed loop control.
     *
     * @param desiredState The desired state of the swerve module.
     * @param isOpenLoop Indicates whether to drive in an open loop (Tele-Op) or closed loop (Autonomous driving) state.
     */
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            m_driveCycle.Output = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            m_driveMotor.setControl(m_driveCycle);
        } else {
            m_driveVoltage.Velocity = Conversions.mpsToFalconRPS(desiredState.speedMetersPerSecond,
                    SwerveConstants.MODULE_TYPE.wheelCircumference, 1);
            m_driveVoltage.FeedForward = m_feedforward.calculate(desiredState.speedMetersPerSecond);
            m_driveMotor.setControl(m_driveVoltage);
        }  
    }

    /**
     * Sets the angle of the swerve module based on the desired state's angle.
     *
     * @param desiredState The desired state of the swerve module.
     */
    private void setAngle(SwerveModuleState desiredState) {
        //System.out.println(getAngle());
        /* Prevent rotating module if speed is less then 1%. Prevents jittering when not moving. */
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED * 0.01)) ? m_lastAngle : desiredState.angle;

        m_steerVoltage.Position = angle.getRotations();//Conversions.degreesToFalcon(angle.getDegrees(), AngleConstants.GEAR_RATIO);
        m_steerMotor.setControl(m_steerVoltage);
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
        return Rotation2d.fromRotations(m_steerMotor.getPosition().refresh().getValue());
    }

    /**
     * Retrieves the current absolute rotation of the CANcoder.
     *
     * @return The current absolute rotation of the CANcoder sensor as a Rotation2d.
     */
    public Rotation2d getEncoderAngle(boolean waitForUpdate) {
        if(waitForUpdate) return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().waitForUpdate(0.5).getValue());
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().refresh().getValue());
    }

    /**
     * Resets the angle motor to the absolute position provided by the CANcoder.
     * 
     * Waits for 500 ms to prevent problems with getting the CANcoder angle before it is initialized.
     */
    public void resetToAbsolute() {
        m_steerMotor.setPosition(getEncoderAngle(true).minus(m_angleOffset).getRotations());//Conversions.degreesToFalcon(position.minus(m_angleOffset).getDegrees(), AngleConstants.GEAR_RATIO));
    }

    /**
     * Configures the angle encoder.
     * 
     * Configures the encoder with the specified configuration.
     */
    private void configAngleEncoder() {
        DeviceConfig.configureSwerveEncoder(m_modLocation + " Angle Encoder", m_angleEncoder, DeviceConfig.swerveEncoderConfig(), Constants.LOOP_TIME_HZ);
    }

    /**
     * Configures the steer motor.
     * 
     * Configures the motor with the specified configuration.
     */
    private void configSteerMotor() {
        DeviceConfig.configureTalonFX(m_modLocation + " Steer Motor", m_steerMotor, DeviceConfig.steerFXConfig(), Constants.LOOP_TIME_HZ);
        resetToAbsolute();
    }

    /**
     * Configures the drive motor.
     * 
     * Configures the motor with the specified configuration.
     */
    private void configDriveMotor() {
        DeviceConfig.configureTalonFX(m_modLocation + " Drive Motor", m_driveMotor, DeviceConfig.driveFXConfig(), Constants.LOOP_TIME_HZ);
    }

    public void configDriveMotorPID(ScreamPIDConstants constants) {
        m_driveMotor.getConfigurator().apply(constants.slot0Configs());
    }

    /**
     * Retrieves the current state of the swerve module.
     *
     * @return The state of the swerve module, including the velocity (m/s) and angle.
     */
    public SwerveModuleState getState(boolean refresh) {
        if(refresh) {
            //m_driveVelocity.refresh();
            m_driveMotor.getVelocity().refresh();
        }

        return new SwerveModuleState(m_driveMotor.getVelocity().getValue(), getAngle());
    }

    /**
     * Retrieves the current position of the swerve module.
     *
     * @return The position of the swerve module, consisting of the distance traveled in meters and the angle.
     */
    public SwerveModulePosition getPosition(boolean refresh) {
        if(refresh) {
            m_drivePosition.refresh();
            //m_driveVelocity.refresh();
            m_driveMotor.getVelocity().refresh();
            m_steerPosition.refresh();
            m_steerVelocity.refresh();
        }
        
        double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveMotor.getVelocity());
        double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        double distance = driveRotations;
        m_internalState.distanceMeters = distance;
        Rotation2d angle = Rotation2d.fromRotations(angleRotations);
        m_internalState.angle = angle;
        
        return m_internalState;
    }
}