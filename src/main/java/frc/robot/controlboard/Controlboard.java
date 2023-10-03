package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Ports;

/**
 * A utility class that contains button bindings.
 * <p>
 * Controlboard allows easy reference of custom button associations.
 */
public class Controlboard{
    private static final XboxController m_driverController = new XboxController(Ports.DRIVER_PORT);

    private static boolean fieldCentric = true;

    /**
     * Retrieves a Translation2d based on the input from the driver controller.
     *
     * @return A Translation2d representing the movement in the x and y directions.
     */
    public static Translation2d getTranslation() {
        return new Translation2d(-m_driverController.getLeftX(), -m_driverController.getLeftY());
    }

    /**
     * Retrieves the rotation value from the driver controller.
     *
     * @return The rotation value of the driver controller.
     */
    public static double getRotation() {
        return -m_driverController.getRightX();
    }

    /**
     * Retrieves whether to zero the gyro from the driver controller.
     *
     * @return True once when to zero the gyro; false otherwise.
     */
    public static boolean getZeroGyro() {
        return m_driverController.getBackButtonPressed();
    }

    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public static boolean getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        new Trigger(m_driverController::getStartButtonPressed).onTrue(new InstantCommand(() -> fieldCentric =! fieldCentric));
        return fieldCentric;
    }

}
