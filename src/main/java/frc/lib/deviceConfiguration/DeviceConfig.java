package frc.lib.deviceConfiguration;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import frc.lib.deviceConfiguration.ErrorChecker.DeviceConfiguration;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants.SwerveConstants;

public class DeviceConfig {

    public DeviceConfig(){}

    public static TalonFXConfiguration genericTalonFXConfig(TalonFX motor, InvertType invert, NeutralMode neutralMode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.setInverted(invert);
        motor.setNeutralMode(neutralMode);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        return config;
    }

    public static void configSwerveDriveFX(TalonFX motor, String name, InvertType invert, NeutralMode neutralMode, ScreamPIDConstants constants){
        TalonFXConfiguration config = genericTalonFXConfig(motor, invert, neutralMode);
        config.slot0.kP = constants.kP();
        config.slot0.kI = constants.kI();
        config.slot0.kD = constants.kD();
        config.slot0.kF = constants.kF();
        config.statorCurrLimit.currentLimit = SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        config.supplyCurrLimit.triggerThresholdCurrent = SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        config.supplyCurrLimit.triggerThresholdTime = SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;
        config.statorCurrLimit.enable = SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        config.openloopRamp = SwerveConstants.OPEN_LOOP_RAMP;
        config.closedloopRamp = SwerveConstants.CLOSED_LOOP_RAMP;
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                motor.configAllSettings(config),
                motor.setSelectedSensorPosition(0));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " Drive Motor ID: " + motor.getDeviceID(), true);
    }

    public static void configSwerveAngleFX(TalonFX motor, String name, InvertType invert, NeutralMode neutralMode, ScreamPIDConstants constants){
        TalonFXConfiguration config = genericTalonFXConfig(motor, invert, neutralMode);
        config.slot0.kP = constants.kP();
        config.slot0.kI = constants.kI();
        config.slot0.kD = constants.kD();
        config.slot0.kF = constants.kF();
        config.statorCurrLimit.currentLimit = SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT;
        config.supplyCurrLimit.triggerThresholdCurrent = SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT;
        config.supplyCurrLimit.triggerThresholdTime = SwerveConstants.ANGLE_PEAK_CURRENT_DURATION;
        config.statorCurrLimit.enable = SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                motor.configAllSettings(config),
                motor.setSelectedSensorPosition(0));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " Angle Motor ID: " + motor.getDeviceID(), true);
    }

    public static void configSwerveCANCoder(CANCoder encoder, String name, boolean invert){
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = invert;
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                encoder.configAllSettings(config));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " CANCoder ID: " + encoder.getDeviceID(), true);
    }
}
