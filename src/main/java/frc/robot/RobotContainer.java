package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.Autonomous;
import frc.robot.auto.Autonomous.NamedCommand;
import frc.robot.auto.Routines;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.controlboard.Controlboard;
import frc.robot.shuffleboard.ShuffleboardTabManager;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    /* Subsystems */
    private static final Swerve m_swerve = new Swerve();
    
    /**
     * Configures the basic robot systems, such as Shuffleboard, autonomous, default commands, and button bindings.
     */
    public RobotContainer() {
        ShuffleboardTabManager.addTabs(true);
        configButtonBindings();
        configDefaultCommands();
        configAuto();
    }

    /**
     * Configures button bindings from Controlboard.
     */
    private void configButtonBindings() {
        Controlboard.getZeroGyro().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    }

    private void configDefaultCommands() { 
        /* Sets the default command for the swerve subsystem */
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve,
                Controlboard.getTranslationY(),
                Controlboard.getTranslationX(),
                Controlboard.getRotation(),
                Controlboard.getFieldCentric()
            )
        );
    }

    /**
     * Configures auto. <p>
     * Configure named commands with {@code}configure(NamedCommand... namedCommands){@code} <p> <STRONG>THIS MUST BE DONE FIRST</STRONG> <p>
     * The default command will automatically be set to Commands.none(). 
     * Use {@code}configure(Command defaultCommand, NamedCommand... namedCommands){@code} to set a custom one.
     * Add auto routines with {@code}addCommands(Command... commands){@code}
     */
    private void configAuto() {
        Autonomous.configure(
            new NamedCommand("ExampleEvent", new PrintCommand("This is an example event :)"))
        );

        Autonomous.addRoutines(
            Routines.exampleAuto().withName("Example Auto")
        );
    }

    /**
     * Retrieves the selected autonomous command from the autonomous chooser.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        System.out.println("Selected auto routine: " + Autonomous.getSelected().getName());
        return Autonomous.getSelected();
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
