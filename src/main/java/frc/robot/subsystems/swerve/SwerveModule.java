package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.deviceConfiguration.DeviceConfig;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

/**
 * A swerve module, which consists of an angle motor, a drive motor, and an angle encoder.
 * <p>
 * This class provides methods to get and set the angle and speed of the module.
 */
public class SwerveModule {
    private int m_moduleNumber;
    private String m_location;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV,
            SwerveConstants.DRIVE_KA);

    /**
     * Constructs a SwerveModule object with the given location, module number, and module constants.
     *<p>
     * @param location The location of the SwerveModule.
     * @param moduleNumber The module number of the SwerveModule.
     * @param moduleConstants The constants specific to this SwerveModule.
     */
    public SwerveModule(String location, int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.m_moduleNumber = moduleNumber;
        this.m_location = location;
        this.m_angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        m_angleEncoder = new CANCoder(moduleConstants.cancoderID, Ports.CANIVORE_BUS_NAME); //TODO delete CANIVORE_BUS_NAME if the robot is not using a CANivore
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
     *<p>
     * @return The location.
     */
    public String getLocation() {
        return m_location;
    }

    /**
     * Returns the module number.
     *<p>
     * @return The module number.
     */
    public int getModuleNumber() {
        return m_moduleNumber;
    }

    /**
     * Sets the neutral mode for both the angle and the drive motor.
     *<p>
     * @param mode The neutral mode to set.
     */
    public void setNeutralMode(NeutralMode mode) {
        m_angleMotor.setNeutralMode(mode);
        m_driveMotor.setNeutralMode(mode);
    }

    /**
     * Sets the desired state of the Swerve module, including the angle and the speed
     *<p>
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
     *<p>
     * @param desiredState The desired state of the swerve module.
     * @param isOpenLoop   A boolean indicating whether the module is in open loop (Tele-Op driving), or closed loop (Autonomous driving).
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    SwerveConstants.WHEEL_CIRCUMFERENCE, SwerveConstants.DRIVE_GEAR_RATIO);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    /**
     * Sets the angle of the swerve module based on the desired state's angle.
     *<p>
     * @param desiredState The desired state of the swerve module.
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_SPEED * 0.01))
                ? m_lastAngle
                : desiredState.angle; /* Prevent rotating module if speed is less then 1%. Prevents jittering when not moving. */
        m_angleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO));
        m_lastAngle = angle;
    }

    /**
     * Optimizes the movement of the module to travel the least distance to get to the desired angle.
     *<p>
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
     * Retrieves the current angle of the rotation motor.
     *<p>
     * @return The angle of the rotation motor in degrees.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), SwerveConstants.ANGLE_GEAR_RATIO));
    }

    /**
     * Retrieves the current absolute rotation of the Cancoder.
     *<p>
     * @return The current absolute rotation of the Cancoder sensor, adjusted by the angle offset, in degrees.
     */
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition()).minus(m_angleOffset);
    }

    /**
     * Resets the angle motor to the absolute position provided by the Cancoder.
     * <p>
     * Delays for 1 second to prevent problems with getting the Cancoder angle before it is initialized.
     */
    public void resetToAbsolute() {
        Timer.delay(1.0);
        m_angleMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(getCanCoder().getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO));
    }

    /**
     * Configures the angle encoder for the swerve module.
     * <p>
     * Sets the angle encoder to its factory default settings and applies the provided configuration settings.
     */
    private void configAngleEncoder() {
        DeviceConfig.configSwerveCANCoder(
            m_angleEncoder, 
            m_location,
            SwerveConstants.CANCODER_INVERT);
    }

    /**
     * Configures the angle motor for swerve drive.
     * <p>
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
     * <p>
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
                Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), SwerveConstants.WHEEL_CIRCUMFERENCE,
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
                Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), SwerveConstants.WHEEL_CIRCUMFERENCE,
                        SwerveConstants.DRIVE_GEAR_RATIO),
                getAngle());
    }

    /**
     * Calculates and returns the distance traveled by the drive motor in meters.
     *
     * @return The distance traveled in meters.
     */
    public double getDistanceMeters() {
        return Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), SwerveConstants.WHEEL_CIRCUMFERENCE,
                SwerveConstants.DRIVE_GEAR_RATIO);
    }
}