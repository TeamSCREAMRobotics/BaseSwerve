package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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
     * <p>
     * This method applies a deadband to the translation, strafe, and rotation values
     * and then passes them to the swerve drive subsystem to drive the robot.
     * 
     * @param translationSup A supplier that provides the translation value
     * @param strafeSup A supplier that provides the strafe value
     * @param rotationSup A supplier that provides the rotation value
     * @param robotCentricSup A supplier that provides a boolean indicating whether the robot should be in robot-centric mode
     */
    @Override
    public void execute() {
        
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        m_swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed),
                rotationVal * SwerveConstants.maxAngularVelocity,
                robotCentricSup.getAsBoolean(),
                true);
    }
}