package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A utility class that contains button bindings.
 * <p>
 * Controlboard allows easy reference of custom button associations.
 */
public class Controlboard extends SubsystemBase {
    private final XboxController m_driverController = new XboxController(0);

    /**
     * Retrieves a Translation2d based on the input from the driver controller.
     *
     * @return A Translation2d representing the movement in the x and y directions.
     */
    public Translation2d getTranslation() {
        return new Translation2d(-m_driverController.getLeftX(), -m_driverController.getLeftY());
    }

    /**
     * Retrieves the rotation value from the driver controller.
     *
     * @return The rotation value of the driver controller.
     */
    public double getRotation() {
        return -m_driverController.getRightX();
    }

    /**
     * Retrieves whether to zero the gyro from the driver controller.
     *
     * @return True once when to zero the gyro; false otherwise.
     */
    public boolean getZeroGyro() {
        return m_driverController.getBackButtonPressed();
    }

    boolean fieldCentric = true;
    /**
     * Retreives the current field-centric mode.
     *
     * @return True if field-centric; false if robot-centric
     */
    public boolean getFieldCentric() {
        /* Toggles field-centric mode between true and false when the start button is pressed */
        new Trigger(m_driverController::getStartButtonPressed).onTrue(new InstantCommand(() -> fieldCentric =! fieldCentric));
        return fieldCentric;
    }

}
