package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that controls the swerve drive system.
 */
public class TeleopSwerve extends CommandBase {
    private Swerve m_swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double lastAngle;
    private Timer holdTimer = new Timer();


    /**
     * Constructs a TeleopSwerve command with the given parameters.
     *
     * @param swerve The Swerve subsystem to control.
     * @param translationSup A supplier for the translation value.
     * @param strafeSup A supplier for the strafe value.
     * @param rotationSup A supplier for the rotation value.
     * @param robotCentricSup A supplier for the robot-centric mode value.
     */
    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.m_swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    /**
     * Executes the swerve drive command.
     * <p>This method applies a deadband to the translation, strafe, and rotation values
     * and then passes them to the swerve drive subsystem to drive the robot.
     */
    @Override
    public void execute() {
        
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        double angularVel = getAngularVelocity(rotationVal);

        m_swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED),
                angularVel,
                robotCentricSup.getAsBoolean(),
                true);
    }

    private double getAngularVelocity(double currentVelocity){
        boolean isRotating = Math.abs(currentVelocity) > 0; 
        if(isRotating){
            holdTimer.reset();
            lastAngle = m_swerve.getYaw().getDegrees();
            return currentVelocity;
        } else { 
            holdTimer.start();
            if(holdTimer.hasElapsed(0.1)){
                return m_swerve.calculateHold(m_swerve.getYaw().getDegrees(), lastAngle);
            }
            return currentVelocity;
        }
    }
}
