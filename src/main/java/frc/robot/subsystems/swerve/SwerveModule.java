package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.CTREConfigs;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    private int m_moduleNumber;
    private String m_name;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    private CTREConfigs m_configs;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV,
            SwerveConstants.driveKA);

    public SwerveModule(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.m_moduleNumber = moduleNumber;
        this.m_name = name;
        this.m_angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        m_angleEncoder = new CANCoder(moduleConstants.cancoderID, Ports.canivoreBusName);
        configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.angleMotorID, Ports.canivoreBusName);
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.driveMotorID, Ports.canivoreBusName);
        configDriveMotor();

        m_lastAngle = getState().angle;
        
        resetToAbsolute();
    }

    public String getName() {
        return m_name;
    }

    public int getModuleNumber() {
        return m_moduleNumber;
    }

    public void setNeutralMode(NeutralMode mode) {
        m_angleMotor.setNeutralMode(mode);
        m_driveMotor.setNeutralMode(mode);
    }

    public void set(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? m_lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        m_angleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.angleGearRatio));
        m_lastAngle = angle;
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
		double deltaDegrees = desiredState.angle.minus(currentAngle).getDegrees();
		boolean invert = Math.abs(deltaDegrees) > 90;
		if(invert) deltaDegrees -= Math.signum(deltaDegrees)*180;
		return new SwerveModuleState(
			(invert ? -1 : 1) * desiredState.speedMetersPerSecond,
			Rotation2d.fromDegrees(currentAngle.getDegrees() + deltaDegrees)
		);
	}

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), SwerveConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition()).minus(m_angleOffset);
    }

    public void resetToAbsolute() {
        Timer.delay(1.0);
        m_angleMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(getCanCoder().getDegrees(), SwerveConstants.angleGearRatio));
    }

    private void configAngleEncoder() {
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(m_configs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(m_configs.swerveAngleFXConfig);
        m_angleMotor.setInverted(SwerveConstants.angleMotorInvert);
        m_angleMotor.setNeutralMode(SwerveConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(m_configs.swerveDriveFXConfig);
        m_driveMotor.setInverted(SwerveConstants.driveMotorInvert);
        m_driveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), SwerveConstants.wheelCircumference,
                        SwerveConstants.driveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference,
                        SwerveConstants.driveGearRatio),
                getAngle());
    }

    public double getDistanceMeters() {
        return Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference,
                SwerveConstants.driveGearRatio);
    }
}