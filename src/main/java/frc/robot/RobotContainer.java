package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoEvents;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.*;
import frc.robot.controlboard.Controlboard;
import frc.robot.shuffleboard.ShuffleboardTabManager;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Controlboard */
    private final Controlboard m_controlboard = new Controlboard();

    /* Subsystems */
    private static final Swerve m_swerve = new Swerve();

    /* Auto */
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

    /* Shuffleboard */
    private final ShuffleboardTabManager m_shuffleboardTabManager = new ShuffleboardTabManager();

    /**
     * Configures the subsystems, default commands, button bindings, and autonomous classes.
     */
    public RobotContainer() {
        /* Controlboard */
        configButtonBindings();

        /* Auto */
        configAuto();

        m_swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_swerve,
                        () -> m_controlboard.getTranslation().getY(),
                        () -> m_controlboard.getTranslation().getX(),
                        () -> m_controlboard.getRotation(),
                        () -> m_controlboard.getFieldCentric()));
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
     * Add options with {@code}addOption{@code}
     */
    private void configAuto() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
        autoTab.add(m_autoChooser);
        addAutoEvents();

        m_autoChooser.setDefaultOption("Do Nothing", AutoRoutines.doNothing());
        m_autoChooser.addOption("Example Routine", AutoRoutines.exampleRoutine());
    }

    /**
     * Adds an event to the event map.
     * The event map defines what command will be run with the corresponding event.
     * Reference these events in PathPlanner to trigger the commands along a path.
     */
    private void addAutoEvents(){
        AutoEvents.addEvent("ExampleEvent", new PrintCommand("This is an example event :)"));
    }

    /**
     * Retrieves the selected autonomous command from the autonomous chooser.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        System.out.println("Selected auto routine: " + m_autoChooser.getSelected().getName());
        return m_autoChooser.getSelected();
    }

    /**
     * Retrieves the Swerve subsystem.
     *
     * @return The Swerve subsystem.
     */
    public static Swerve getSwerve() {
        return m_swerve;
    }
}
