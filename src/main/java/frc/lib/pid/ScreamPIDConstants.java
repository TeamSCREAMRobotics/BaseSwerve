package frc.lib.pid;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;

/**
 * A container class for PID constants, along with additional methods.
 */
public class ScreamPIDConstants implements Cloneable{
    private double kP, kI, kD, kF = 0;
    private double period = 0.02;
    private double minOutput = -1;
    private double maxOutput = 1;
    private double integralZone = Double.MAX_VALUE;
    private double maxIntegralAccumulator = Double.POSITIVE_INFINITY;
    private double minIntegralAccumulator = Double.NEGATIVE_INFINITY;

    public ScreamPIDConstants(){}

    public ScreamPIDConstants(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public ScreamPIDConstants(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

	public void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setPIDF(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setPeriod(double period){
        this.period = period;
    }

    public void setkP(double kP){
        this.kP = kP;
    }
    
    public void setkI(double kI){
        this.kI = kI;
    }
    
    public void setkD(double kD){
        this.kD = kD;
    }

    public void setkF(double kF){
        this.kF = kF;
    }

    public void setIntegralZone(double Izone){
        this.integralZone = Izone;
    }

    public void setIntegralAccumulatorBounds(double max, double min){
        this.maxIntegralAccumulator = max;
        this.minIntegralAccumulator = min;
    }

    public void setOutputBounds(double max, double min){
        this.maxOutput = max;
        this.minOutput = min;
    }

    public double period(){
        return period;
    }

    public double kP(){
        return kP;
    }

    public double kI(){
        return kI;
    }

    public double kD(){
        return kD;
    }

    public double kF(){
        return kF;
    }

    public double integralZone(){
        return integralZone;
    }

    public double maxIntegralAccumulator(){
        return maxIntegralAccumulator;
    }

    public double minIntegralAccumulator(){
        return minIntegralAccumulator;
    }

    public double maxOutput(){
        return maxOutput;
    }

    public double minOutput(){
        return minOutput;
    }

    public Slot0Configs slot0Configs(){
        Slot0Configs config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kF;
        return config;
    }

    public Slot1Configs slot1Configs(){
        Slot1Configs config = new Slot1Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kF;
        return config;
    }

    public Slot2Configs slot2Configs(){
        Slot2Configs config = new Slot2Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kF;
        return config;
    }

    public boolean equals(ScreamPIDConstants other){
        return
            this.period == other.period &&
            this.kP == other.kP &&
            this.kI == other.kI &&
            this.kD == other.kD &&
            this.kF == other.kF &&
            this.minOutput == other.minOutput &&
            this.maxOutput == other.maxOutput &&
            this.integralZone == other.integralZone &&
            this.maxIntegralAccumulator == other.maxIntegralAccumulator &&
            this.minIntegralAccumulator == other.minIntegralAccumulator;
    }

    public ScreamPIDConstants clone(){
        ScreamPIDConstants copy = new ScreamPIDConstants();
        copy.period = this.period;
        copy.kP = this.kP;
        copy.kI = this.kI;
        copy.kD = this.kD;
        copy.kF = this.kF;
        copy.minOutput = this.minOutput;
        copy.maxOutput = this.maxOutput;
        copy.integralZone = this.integralZone;
        copy.maxIntegralAccumulator = this.maxIntegralAccumulator;
        copy.minIntegralAccumulator = this.minIntegralAccumulator;
        return copy;
    }
}
