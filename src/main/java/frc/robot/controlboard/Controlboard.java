package frc.robot.controlboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A utility class that contains button bindings.
 * 
 * Controlboard allows easy reference of custom button associations.
 */
public class Controlboard{

    public static final double STICK_DEADBAND = 0.15;

    private static final XboxController driverController = new XboxController(0);

    private static boolean fieldCentric = true;

    /**
     * Retrieves a Translation2d based on the input from the driver controller.
     *
     * @return A Translation2d representing the movement in the x and y directions.
     */
    public static Translation2d getTranslation() {
        return new Translation2d(-MathUtil.applyDeadband(driverController.getLeftX(), STICK_DEADBAND), -MathUtil.applyDeadband(driverController.getLeftY(), STICK_DEADBAND));
    }

    /**
     * Retrieves the rotation value from the driver controller.
     *
     * @return The rotation value of the driver controller.
     */
    public static double getRotation() {
        return -MathUtil.applyDeadband(driverController.getRightX(), STICK_DEADBAND);
    }

    /**
     * Retrieves whether to zero the gyro from the driver controller.
     *
     * @return True once when to zero the gyro; false otherwise.
     */
    public static boolean getZeroGyro() {
        return driverController.getBackButtonPressed();
    }

    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public static boolean getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        new Trigger(driverController::getStartButtonPressed).onTrue(new InstantCommand(() -> fieldCentric =! fieldCentric));
        return fieldCentric;
    }

}
