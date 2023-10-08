package frc.lib.deviceConfiguration;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.deviceConfiguration.ErrorChecker.DeviceConfiguration;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants.SwerveConstants;

public class DeviceConfig {

    public DeviceConfig(){}

    public static TalonFXConfiguration genericTalonFXConfig(InvertedValue invert, NeutralModeValue neutralMode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invert;
        config.MotorOutput.NeutralMode = neutralMode;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        return config;
    }

    public static void configSwerveDriveFX(TalonFX motor, String name, InvertedValue invert, NeutralModeValue neutralMode, ScreamPIDConstants constants){
        TalonFXConfiguration config = genericTalonFXConfig(invert, neutralMode);
        config.Slot0.kP = constants.kP();
        config.Slot0.kI = constants.kI();
        config.Slot0.kD = constants.kD();
        config.Slot0.kV = constants.kF();
        config.CurrentLimits.StatorCurrentLimit = SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;
        config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.OPEN_LOOP_RAMP;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.CLOSED_LOOP_RAMP;
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                motor.getConfigurator().apply(config),
                motor.setRotorPosition(0));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " Drive Motor ID: " + motor.getDeviceID(), true);
    }

    public static void configSwerveAngleFX(TalonFX motor, String name, InvertedValue invert, NeutralModeValue neutralMode, ScreamPIDConstants constants){
        TalonFXConfiguration config = genericTalonFXConfig(invert, neutralMode);
        config.Slot0.kP = constants.kP();
        config.Slot0.kI = constants.kI();
        config.Slot0.kD = constants.kD();
        config.Slot0.kV = constants.kF();
        config.CurrentLimits.StatorCurrentLimit = SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.ANGLE_PEAK_CURRENT_DURATION;
        config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                motor.getConfigurator().apply(config),
                motor.setRotorPosition(0));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " Angle Motor ID: " + motor.getDeviceID(), true);
    }

    public static void configSwerveCANcoder(CANcoder encoder, String name, SensorDirectionValue invert){
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection = invert;
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                encoder.getConfigurator().apply(config));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " CANcoder ID: " + encoder.getDeviceID(), true);
    }
}
