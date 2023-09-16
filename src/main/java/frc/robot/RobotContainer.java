package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private final AutoRoutines m_autoRoutines;

    /* Shuffleboard */
    private final ShuffleboardTabManager m_shuffleboardTabManager;
    private ShuffleboardTab m_autoTab;

    /**
     * Configures the subsystems, default commands, button bindings, and the autonomous chooser.
     */
    public RobotContainer() {
        /* Controlboard */
        m_controlboard = new Controlboard();

        /* Subsystems */
        m_swerve = new Swerve();

        /* Auto */
        m_autoRoutines = new AutoRoutines(m_swerve);

        /* Shuffleboard */
        m_shuffleboardTabManager = new ShuffleboardTabManager(this);
        m_autoTab = Shuffleboard.getTab("Auto");

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

    /**
     * Configures button bindings using methods from Controlboard.
     */
    private void configButtonBindings() {
        new Trigger(() -> m_controlboard.getZeroGyro()).onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    }

    /**
     * Configures the autonomous chooser on the SmartDashboard.
     * <p>
     * Add options with <STRONG>addOption</STRONG>.
     */
    private void configAutoChooser() {
        m_autoTab.add(m_autoChooser);

        m_autoChooser.setDefaultOption("Do Nothing", m_autoRoutines.doNothing());
        m_autoChooser.addOption("Example Option", m_autoRoutines.exampleRoutine());
    }

    /**
     * Retrieves the selected autonomous command from the autonomous chooser.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    /**
     * Retrieves the Swerve subsystem.
     *
     * @return The Swerve subsystem.
     */
    public Swerve getSwerve() {
        return m_swerve;
    }
}
