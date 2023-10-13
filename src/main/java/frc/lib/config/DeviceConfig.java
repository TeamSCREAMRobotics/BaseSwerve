package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.config.ErrorChecker.DeviceConfiguration;
import frc.lib.pid.ScreamPIDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.AngleConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;

public class DeviceConfig {

    public DeviceConfig(){}

    ////////////////////////////////////// CUSTOM CONFIGURATION METHODS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

    public static TalonFXConfiguration driveFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput = DeviceConfig.FXMotorOutputConfig(DriveConstants.MOTOR_INVERT, DriveConstants.NEUTRAL_MODE);
        config.Feedback = DeviceConfig.FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor);
        config.CurrentLimits = DeviceConfig.FXCurrentLimitsConfig(
            DriveConstants.CURRENT_LIMIT_ENABLE, 
            DriveConstants.SUPPLY_CURRENT_LIMIT, 
            DriveConstants.SUPPLY_CURRENT_THRESHOLD, 
            DriveConstants.SUPPLY_TIME_THRESHOLD);
        config.Slot0 = (Slot0Configs) DeviceConfig.FXPIDConfig(DriveConstants.PID_CONSTANTS);
        config.OpenLoopRamps = DeviceConfig.FXOpenLoopRampConfig(DriveConstants.OPEN_LOOP_RAMP);
        config.ClosedLoopRamps = DeviceConfig.FXClosedLoopRampConfig(DriveConstants.CLOSED_LOOP_RAMP);
        return config;
    }

    public static TalonFXConfiguration angleFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput = DeviceConfig.FXMotorOutputConfig(AngleConstants.MOTOR_INVERT, AngleConstants.NEUTRAL_MODE);
        config.Feedback = DeviceConfig.FXFeedbackConfig(FeedbackSensorSourceValue.RotorSensor, 0, AngleConstants.GEAR_RATIO);
        config.CurrentLimits = DeviceConfig.FXCurrentLimitsConfig(
            AngleConstants.CURRENT_LIMIT_ENABLE, 
            AngleConstants.SUPPLY_CURRENT_LIMIT, 
            AngleConstants.SUPPLY_CURRENT_THRESHOLD, 
            AngleConstants.SUPPLY_TIME_THRESHOLD);
        config.Slot0 = DeviceConfig.FXPIDConfig(AngleConstants.PID_CONSTANTS);
        return config;
    }

    public static CANcoderConfiguration swerveEncoderConfig(){
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.SensorDirection = SwerveConstants.CANCODER_INVERT;
        return config;
    }



    ////////////////////////////////////// GENERIC CONFIGURATION METHODS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

    public static void configureTalonFX(String name, TalonFX motor, TalonFXConfiguration config){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                motor.getConfigurator().apply(config),
                motor.getConfigurator().setPosition(0));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + motor.getDeviceID(), true);
    }

    public static void configureSwerveEncoder(String name, CANcoder encoder, CANcoderConfiguration config){
        DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings(){
            return ErrorChecker.hasConfiguredWithoutErrors(
                encoder.getConfigurator().apply(config),
                encoder.getConfigurator().setPosition(50));
        }
        };
        ErrorChecker.configureDevice(deviceConfig, name + " " + encoder.getDeviceID(), true);
    }

    public static MotorOutputConfigs FXMotorOutputConfig(InvertedValue invert, NeutralModeValue neutralMode){
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.Inverted = invert;
        config.NeutralMode = neutralMode;
        return config;
    }

    public static FeedbackConfigs FXFeedbackConfig(FeedbackSensorSourceValue sensor){
        FeedbackConfigs config = new FeedbackConfigs();
        config.FeedbackSensorSource = sensor;
        return config;
    }

    public static FeedbackConfigs FXFeedbackConfig(FeedbackSensorSourceValue sensor, int remoteSensorID, double gearRatio){
        FeedbackConfigs config = new FeedbackConfigs();
        config.FeedbackSensorSource = sensor;
        config.FeedbackRemoteSensorID = remoteSensorID;
        config.SensorToMechanismRatio = gearRatio;
        return config;
    }

    public static Slot0Configs FXPIDConfig(ScreamPIDConstants constants){
        return constants.slot0Configs();
    }

    public static OpenLoopRampsConfigs FXOpenLoopRampConfig(double ramp){
        OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
        config.DutyCycleOpenLoopRampPeriod = ramp;
        return config;
    }

    public static ClosedLoopRampsConfigs FXClosedLoopRampConfig(double ramp){
        ClosedLoopRampsConfigs config = new ClosedLoopRampsConfigs();
        config.DutyCycleClosedLoopRampPeriod = ramp;
        return config;
    }

    public static CurrentLimitsConfigs FXCurrentLimitsConfig(boolean enable, double limit, double currentThreshold, double timeThreshold){
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = enable;
        config.SupplyCurrentLimit = limit;
        config.SupplyCurrentThreshold = currentThreshold;
        config.SupplyTimeThreshold = timeThreshold;
        return config;
    }
}
