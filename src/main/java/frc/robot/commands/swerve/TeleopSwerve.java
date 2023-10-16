package frc.robot.commands.swerve;

import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that controls the swerve drive system.
 */
public class TeleopSwerve extends Command {
    private Swerve swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldCentricSup;
    private double lastAngle;
    private Timer holdTimer = new Timer();


    /**
     * Constructs a TeleopSwerve command with the given parameters.
     *
     * @param swerve The Swerve subsystem to control.
     * @param translationSup A supplier for the translation value.
     * @param strafeSup A supplier for the strafe value.
     * @param rotationSup A supplier for the rotation value.
     * @param fieldCentricSup A supplier for the drive mode. Robot centric = false; Field centric = true
     */
    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier fieldCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldCentricSup = fieldCentricSup;
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
        double rotationVal = rotationSup.getAsDouble();//getRotation(rotationSup.getAsDouble());
        boolean fieldCentric = fieldCentricSup.getAsBoolean();


        swerve.setChassisSpeeds(
            swerve.robotSpeeds(
                new Translation2d(translationVal, strafeVal), 
                rotationVal, 
                fieldCentric
            ),
            true
        );
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
                return swerve.calculateHeadingCorrection(swerve.getYaw().getDegrees(), lastAngle);
            }
            return current;
        }
    }
}
