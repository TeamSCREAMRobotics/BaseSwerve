package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Auto;
import frc.robot.auto.Routines;
import frc.robot.auto.Auto.NamedCommand;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.controlboard.Controlboard;
import frc.robot.shuffleboard.ShuffleboardTabManager;
import frc.robot.subsystems.*;

public class RobotContainer {

    /* Subsystems */
    private static final Swerve m_swerve = new Swerve();
    
    /**
     * Configures the subsystems, default commands, button bindings, and autonomous classes.
     */
    public RobotContainer() {
        /* Shuffleboard */
        ShuffleboardTabManager.addTabs(true);

        /* Controlboard */
        configButtonBindings();

        /* Auto */
        configAuto();

        m_swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_swerve,
                        Controlboard.getTranslationY(),
                        Controlboard.getTranslationX(),
                        Controlboard.getRotation(),
                        Controlboard.getFieldCentric()));
    }

    /**
     * Configures button bindings using methods from Controlboard.
     */
    private void configButtonBindings() {
        Controlboard.getZeroGyro().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
    }

    /**
     * Configures auto. <p>
     * Add events with {@code}registerEvents(Event... events){@code} <p> <STRONG> MUST BE DONE FIRST</STRONG> <p>
     * Set the default command with {@code}setDefault(Command auto){@code} <p>
     * Add additional commands with {@code}add(Command... autos){@code}
     */
    private void configAuto() {
        Auto.configure(
            Routines.doNothing(), 
            new NamedCommand("ExampleEvent", new PrintCommand("This is an example event :)"))
        );

        Auto.addRoutines(
            //Routines.exampleSequence(),
            //Routines.exampleAuto()
        );
    }

    /**
     * Retrieves the selected autonomous command from the autonomous chooser.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        System.out.println("Selected auto routine: " + Auto.getSelected().getName());
        return Auto.getSelected();
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
