package frc.lib.deviceConfiguration;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.SwerveConstants;

/**
 *  A container class for device configurations. 
 * */
public class DeviceConfigs {
	public static final int maxStatusFramePeriod = 255;
    public static final double kNominalVoltage = 12.6;

    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public DeviceConfigs(){
        configSwerve();
    }

    public static class CANCoderConfig {
        public int sensorDataPeriod = 20;
        public int vBatAndFaultsPeriod = 20;
        public AbsoluteSensorRange absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        public boolean sensorDirection = false;
    }
    
    public static class TalonFXConfig {

        public int status_1_General = 20;
        public int status_2_Feedback0 = 50;
        public int status_3_Quadrature = 50;
        public int status_4_AinTempVbat = 50;
        public int status_6_Misc = maxStatusFramePeriod;
        public int status_7_CommStatus = 20;
        public int status_8_PulseWidth = maxStatusFramePeriod;
        public int status_9_MotProfBuffer = maxStatusFramePeriod;
        public int status_10_Targets = maxStatusFramePeriod;
        public int status_11_UartGadgeteer = maxStatusFramePeriod;
        public int status_12_Feedback1 = maxStatusFramePeriod;
        public int status_13_Base_PIDF0 = 50;
        public int status_14_Turn_PIDF1 = maxStatusFramePeriod;
        public int status_15_FirmwareApiStatus = maxStatusFramePeriod;
        public int status_17_Targets1 = maxStatusFramePeriod;
        public int status_21_FeedbackIntegrated = 50;
        public int status_Brushless_Current = 50;

        public boolean enableVoltageCompensation = true;
        public FeedbackDevice feedbackDevice = FeedbackDevice.IntegratedSensor;
        public StatorCurrentLimitConfiguration statorCurrentLimitConfiguration = new StatorCurrentLimitConfiguration();
        public double neutralDeadband = 0;
        public double voltageCompSaturation = kNominalVoltage;
        public double forwardSoftLimit = Double.MAX_VALUE;
        public double reverseSoftLimit = Double.MIN_VALUE;
        
    }

    public static class TalonSRXConfig{
        public int status_1_General = 50;
		public int status_2_Feedback0 = maxStatusFramePeriod;
		public int status_3_Quadrature = maxStatusFramePeriod;
		public int status_4_AinTempVbat = 50;
		public int status_6_Misc = maxStatusFramePeriod;
		public int status_7_CommStatus = 20;
		public int status_8_PulseWidth = maxStatusFramePeriod;
		public int status_9_MotProfBuffer = maxStatusFramePeriod;
		public int status_10_Targets = maxStatusFramePeriod;
		public int status_11_UartGadgeteer = maxStatusFramePeriod;
		public int status_12_Feedback1 = maxStatusFramePeriod;
		public int status_13_Base_PIDF0 = maxStatusFramePeriod;
		public int status_14_Turn_PIDF1 = maxStatusFramePeriod;
		public int status_15_FirmwareApiStatus = maxStatusFramePeriod;
		public int status_17_Targets1 = maxStatusFramePeriod;
		public int status_21_FeedbackIntegrated = maxStatusFramePeriod;
		public int status_Brushless_Current = 20;
        
        public boolean enableVoltageCompensation = true;
        public FeedbackDevice feedbackDevice = FeedbackDevice.QuadEncoder;	
        public double neutralDeadband = 0;
        public double voltageCompSaturation = kNominalVoltage;
        public double forwardSoftLimit = Double.MAX_VALUE;
        public double reverseSoftLimit = Double.MIN_VALUE;
    }

    public static class Pigeon2Config{
        public int biasedStatus_2_Gyro = maxStatusFramePeriod;
        public int biasedStatus_4_Mag = maxStatusFramePeriod;
        public int biasedStatus_6_Accel = maxStatusFramePeriod;
        public int condStatus_10_SixDeg_Quat = maxStatusFramePeriod;
        public int condStatus_11_GyroAccum = maxStatusFramePeriod;
        public int condStatus_1_General = 20;
        public int condStatus_2_GeneralCompass = maxStatusFramePeriod;
        public int condStatus_3_GeneralAccel = maxStatusFramePeriod;
        public int condStatus_6_SensorFusion = maxStatusFramePeriod;
        public int condStatus_9_SixDeg_YPR = 20;
        public int rawStatus_4_Mag = maxStatusFramePeriod;
    }

    public static class CANSparkMaxConfig{
        public int status0 = 10;
        public int status1 = 50;
        public int status2 = 20;
        public int status3 = 50;
        public int status4 = maxStatusFramePeriod;
        public int status5 = maxStatusFramePeriod;
        public int status6 = maxStatusFramePeriod;

        public boolean enableVoltageCompensation = true;
        public double voltageCompSaturation = kNominalVoltage;
        public IdleMode idleMode = IdleMode.kCoast;
    }

    public void configSwerve() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        // Swerve Angle Motor Configurations
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                SwerveConstants.angleEnableCurrentLimit,
                SwerveConstants.angleContinuousCurrentLimit,
                SwerveConstants.anglePeakCurrentLimit,
                SwerveConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = SwerveConstants.angleKD;
        swerveAngleFXConfig.slot0.kF = SwerveConstants.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        // Swerve Drive Motor Configuration
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                SwerveConstants.driveEnableCurrentLimit,
                SwerveConstants.driveContinuousCurrentLimit,
                SwerveConstants.drivePeakCurrentLimit,
                SwerveConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = SwerveConstants.driveKF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveConstants.closedLoopRamp;

        // Swerve CANCoder Configuration
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
