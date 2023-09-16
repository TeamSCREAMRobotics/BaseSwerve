package frc.robot.auto;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Swerve;

/**
 * A utility class that holds all events for use during the autonomous period.
 */
public class AutoEvents {
    
    private static HashMap<String, Command> m_autoEvents = new HashMap<String, Command>();

    public AutoEvents(Swerve swerve){
        addEvent("Example", new PrintCommand("This is an example event :)"));
    }

    public static void addEvent(String key, Command command){
        m_autoEvents.put(key, command);
    }

    public static void removeEvents(String... keys){
        for(String key : keys){
            m_autoEvents.remove(key);
        }
    }

    public static HashMap<String, Command> getEvents(){
        return m_autoEvents;
    }
}
