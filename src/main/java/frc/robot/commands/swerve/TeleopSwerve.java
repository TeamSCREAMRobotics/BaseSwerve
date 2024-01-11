package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.controlboard.Controlboard;
import frc.robot.subsystems.swerve.Swerve;

/**
 * A command that controls the swerve drive system.
 */
public class TeleopSwerve extends Command {
    private Swerve swerve;
    private DoubleSupplier[] translationSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;
    private Rotation2d lastAngle;
    private Timer correctionTimer = new Timer();


    /**
     * Constructs a TeleopSwerve command with the given parameters.
     *
     * @param swerve The Swerve subsystem to control.
     * @param translationSup A supplier array for the translation value.
     * @param strafeSup A supplier for the strafe value.
     * @param rotationSup A supplier for the rotation value.
     * @param fieldRelativeSup A supplier for the drive mode. Robot relative = false; Field relative = true
     */
    public TeleopSwerve(Swerve swerve, DoubleSupplier[] translationSup, DoubleSupplier rotationSup, BooleanSupplier fieldRelativeSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldRelativeSup;
    }

    @Override
    public void initialize() {
        correctionTimer.stop();
        correctionTimer.reset();
        lastAngle = swerve.getYaw();
    } 

    /**
     * Executes the swerve drive command.
     * Passes the translation, strafe, and rotation values to the swerve subsystem to drive the robot.
     */
    @Override
    public void execute() {
        
        Translation2d translationVal = new Translation2d(translationSup[0].getAsDouble(), translationSup[1].getAsDouble()).times(SwerveConstants.MAX_SPEED);
        double rotationVal = getRotation(rotationSup.getAsDouble());
        boolean fieldRelativeVal = fieldRelativeSup.getAsBoolean();

        if(Controlboard.getZeroGyro().getAsBoolean()) lastAngle = Rotation2d.fromDegrees(0.0);

        swerve.setChassisSpeeds(
            fieldRelativeVal ? swerve.fieldRelativeSpeeds(translationVal, rotationVal) : swerve.robotRelativeSpeeds(translationVal, rotationVal),
            true
        );
    }

    /**
     * Checks if the swerve drive should start heading correction.<p>
     * If no manual input is given for a time, this method will return the rotation value required to maintain the current heading.
     * 
     * @param current The current rotation value.
     * @return The determined rotation value.
     */
    private double getRotation(double current){
        boolean rotating = Math.abs(current) > 0;

        if(rotating){
            correctionTimer.reset();
            return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
        }

        correctionTimer.start();

        if(correctionTimer.get() <= SwerveConstants.CORRECTION_TIME_THRESHOLD){
            lastAngle = swerve.getYaw();
        }

        if(correctionTimer.hasElapsed(SwerveConstants.CORRECTION_TIME_THRESHOLD)){
            return swerve.calculateHeadingCorrection(swerve.getYaw().getDegrees(), lastAngle.getDegrees());
        }

        return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
    }
}
