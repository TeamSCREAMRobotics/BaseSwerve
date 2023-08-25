package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.*;
import frc.robot.controlboard.Controlboard;
import frc.robot.shuffleboard.ShuffleboardTabManager;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Controlboard */
    private final Controlboard m_controlboard;

    /* Subsystems */
    private final Swerve m_swerve;

    /* Auto */
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

    /* Shuffleboard */
    private final ShuffleboardTabManager m_shuffleboardTabManager;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_controlboard = new Controlboard();
        m_swerve = new Swerve();
        m_shuffleboardTabManager = new ShuffleboardTabManager(this);

        m_swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_swerve,
                        () -> m_controlboard.getTranslation().getY(),
                        () -> m_controlboard.getTranslation().getX(),
                        () -> m_controlboard.getRotation(),
                        () -> m_controlboard.getFieldCentric()));

        configButtonBindings();
        configAutoChooser();
    }

    private void configButtonBindings() {
        new Trigger(() -> m_controlboard.getZeroGyro()).onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    }

    private void configAutoChooser() {
        SmartDashboard.putData(m_autoChooser);

        m_autoChooser.addOption("Example Option", AutoRoutines.exampleRoutine(m_swerve));
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public Swerve getSwerve() {
        return m_swerve;
    }
}
