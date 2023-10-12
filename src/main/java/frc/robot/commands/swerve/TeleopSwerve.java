package frc.robot.commands.swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that controls the swerve drive system.
 */
public class TeleopSwerve extends CommandBase {
    private Swerve swerve;
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
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        lastAngle = swerve.getYaw().getDegrees();
    }

    /**
     * Executes the swerve drive command.
     * <p>This method applies a deadband to the translation, strafe, and rotation values
     * and then passes them to the swerve drive subsystem to drive the robot.
     */
    @Override
    public void execute() {
        
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = getRotation(rotationSup.getAsDouble());

        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED),
                rotationVal,
                robotCentricSup.getAsBoolean(),
                true);
    }

    private double getRotation(double current){
        boolean isRotating = Math.abs(current) > 0;
        if(isRotating){
            holdTimer.reset();
            lastAngle = swerve.getYaw().getDegrees();
            return current;
        } else { 
            holdTimer.start();
            if(holdTimer.hasElapsed(0.1)){
                return swerve.calculateHold(swerve.getYaw().getDegrees(), lastAngle);
            }
            return current;
        }
    }
}
