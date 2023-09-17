package frc.lib.deviceConfiguration;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.deviceConfiguration.ErrorChecker.DeviceConfiguration;
import frc.lib.pid.MotionMagicConstants;
import frc.lib.pid.ScreamPIDConstants;

/**
 * Utilities for configuring various motor controllers.
 */
public class DeviceUtil {
    
    /**
     * Configures the PID constants for a TalonFX motor controller.
     *
     * @param motor The TalonFX motor controller to configure.
     * @param pidConstants The PID constants to set for the controller.
     * @param printInfo Whether to print information about the configuration process.
     */
    public static void configTalonFXPID(TalonFX motor, ScreamPIDConstants pidConstants, boolean printInfo){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
                    motor.config_kP(0, pidConstants.kP()),
					motor.config_kI(0, pidConstants.kI()),
					motor.config_kD(0, pidConstants.kD()),
					motor.config_kF(0, pidConstants.kF()),
					motor.config_IntegralZone(0, pidConstants.integralZone()),
					motor.configMaxIntegralAccumulator(0, pidConstants.maxIntegralAccumulator()),
					motor.configClosedLoopPeakOutput(0, pidConstants.maxOutput())
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonFXPID", printInfo);
	}
	
 	/**
  	* Configures the PID constants for a TalonSRX motor controller.
  	*
  	* @param motor The TalonSRX motor controller to configure.
  	* @param pidConstants The PID constants to set for the controller.
  	* @param printInfo Whether to print information about the configuration process.
  	*/
	public static void configTalonSRXPID(TalonSRX motor, ScreamPIDConstants pidConstants, boolean printInfo){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					motor.config_kP(0, pidConstants.kP()),
					motor.config_kI(0, pidConstants.kI()),
					motor.config_kD(0, pidConstants.kD()),
					motor.config_kF(0, pidConstants.kF()),
					motor.config_IntegralZone(0, pidConstants.integralZone()),
					motor.configMaxIntegralAccumulator(0, pidConstants.maxIntegralAccumulator()),
					motor.configClosedLoopPeakOutput(0, pidConstants.maxOutput())
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonSRXPID", printInfo);
	}

 	/**
  	* Configures TalonFX MotionMagic with the provided constants.
  	*
  	* @param motor The TalonFX motor to configure.
  	* @param motionMagicConstants The motion magic constants to set for the motor.
  	* @param printInfo Whether to print information about the configuration process.
  	*/
	public static void configTalonFXMotionMagic(TalonFX motor, MotionMagicConstants motionMagicConstants, boolean printInfo){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					motor.configMotionCruiseVelocity(motionMagicConstants.cruiseVelocity),
					motor.configMotionAcceleration(motionMagicConstants.acceleration),
					motor.configMotionSCurveStrength(motionMagicConstants.sCurveStrength)
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonFXMotionMagic", printInfo);
	}

 	/**
  	* Configures TalonSRX MotionMagic with the provided constants.
  	*
  	* @param motor The TalonSRX motor to configure.
  	* @param motionMagicConstants The motion magic constants to set for the motor.
  	* @param printInfo Whether to print information about the configuration process.
  	*/
	public static void configTalonSRXMotionMagic(TalonSRX motor, MotionMagicConstants motionMagicConstants, boolean printInfo){

		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					motor.configMotionCruiseVelocity(motionMagicConstants.cruiseVelocity),
					motor.configMotionAcceleration(motionMagicConstants.acceleration),
					motor.configMotionSCurveStrength(motionMagicConstants.sCurveStrength)
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonSRXMotionMagic", printInfo);
	}
}