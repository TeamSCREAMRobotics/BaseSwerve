package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A utility class that contains predefined auto routines for use during the autonomous period.
 */
public class Auto{

    private static final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
    public record NamedCommand(String name, Command command){}

    public static void configure(Command defaultAuto, NamedCommand... events){
        Shuffleboard.getTab("Auto").add("Selected Auto", m_autoChooser).withSize(2, 1);
        for(NamedCommand event : events){
            NamedCommands.registerCommand(event.name(), event.command());
        }
        m_autoChooser.setDefaultOption(defaultAuto.getName(), defaultAuto);
    }

    public static Command getSelected(){
        return m_autoChooser.getSelected();
    }

    public static void addRoutines(Command... autos){
        
        for(Command auto : autos){
            m_autoChooser.addOption(auto.getName(), auto);
        }
    }
}
