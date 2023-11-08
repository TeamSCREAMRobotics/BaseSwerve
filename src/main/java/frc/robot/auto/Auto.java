package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A utility class that contains predefined auto routines for use during the autonomous period.
 */
public class Auto{

    private static SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
    public record NamedCommand(String name, Command command){}

    public static void configure(NamedCommand... namedCommands){
        for(NamedCommand command : namedCommands){
            NamedCommands.registerCommand(command.name(), command.command());
        }
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
        m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
    }

    public static Command getSelected(){
        return m_autoChooser.getSelected();
    }

    public static void addCommands(Command... commands){
        for(Command command : commands){
            m_autoChooser.addOption(command.getName(), command);
        }
    }
}
