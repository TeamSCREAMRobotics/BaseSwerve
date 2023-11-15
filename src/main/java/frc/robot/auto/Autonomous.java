package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A utility class that contains predefined auto routines for use during the autonomous period.
 */
public class Autonomous{

    private static SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

    public record NamedCommand(String name, Command command){}

    private static boolean configured = false;

    public static void configure(Command defaultCommand, NamedCommand... namedCommands){
        if(configured){
            DriverStation.reportWarning("Auto already configured!", true);
        }

        for(NamedCommand command : namedCommands){
            NamedCommands.registerCommand(command.name(), command.command());
        }

        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        m_autoChooser.setDefaultOption("Do Nothing", defaultCommand);
        configured = true;
    }

    public static Command getSelected(){
        return m_autoChooser.getSelected();
    }

    public static void addRoutines(Command... routines){
        for(Command routine : routines){
            m_autoChooser.addOption(routine.getName(), routine);
        }
    }
}
