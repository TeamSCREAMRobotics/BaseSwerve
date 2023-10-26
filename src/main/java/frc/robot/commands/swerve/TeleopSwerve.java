package frc.robot.commands.swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.controlboard.Controlboard;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private Rotation2d lastAngle;
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
    }

    @Override
    public void initialize() {
        lastAngle = swerve.getYaw();
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
        boolean fieldCentric = fieldCentricSup.getAsBoolean();

        if(Controlboard.getZeroGyro().getAsBoolean()) lastAngle = Rotation2d.fromDegrees(0);

        swerve.setChassisSpeeds(
            swerve.robotSpeeds(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED), 
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
            return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
        } else { 
            holdTimer.start();
            if(holdTimer.get() < 0.2){
                lastAngle = swerve.getYaw();
            }
            if(holdTimer.hasElapsed(0.2)){
                return swerve.calculateHeadingCorrection(swerve.getYaw().getDegrees(), lastAngle.getDegrees());
            }
            return current * SwerveConstants.MAX_ANGULAR_VELOCITY;
        }
    }
}
