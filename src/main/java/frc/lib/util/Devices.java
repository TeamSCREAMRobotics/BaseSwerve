package frc.lib.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Devices {

    public static TalonSRX genericTalonSRX(int id, boolean doInvert) {
        TalonSRX motor = new TalonSRX(id);
        motor.clearStickyFaults();
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(doInvert);
        return motor;
    }

    public static TalonFX genericTalonFX(int id, boolean doInvert) {
        TalonFX motor = new TalonFX(id);
        motor.clearStickyFaults();
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(doInvert);
        return motor;
    }
}
