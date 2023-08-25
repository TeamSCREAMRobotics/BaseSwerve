package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Controlboard extends SubsystemBase {
    private final XboxController m_driverController = new XboxController(0);

    public Translation2d getTranslation() {
        return new Translation2d(-m_driverController.getLeftX(), -m_driverController.getLeftY());
    }

    public double getRotation() {
        return -m_driverController.getRightX();
    }

    public boolean getZeroGyro() {
        return m_driverController.getBackButtonPressed();
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    boolean fieldCentric = true;

    @Override
    public void periodic() {
        if (m_driverController.getStartButtonPressed())
            fieldCentric = !fieldCentric;
    }

}
