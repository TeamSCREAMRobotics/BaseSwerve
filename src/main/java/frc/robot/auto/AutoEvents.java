package frc.robot.auto;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;

/**
 * A utility class that holds all events for use during the autonomous period.
 */
public class AutoEvents extends CommandBase{
    
    private static final HashMap<String, Command> m_autoEventMap = new HashMap<String, Command>();
    
    public static void addEvent(String key, Command command){
        m_autoEventMap.put(key, command);
    }

    public static void addEventMap(HashMap<String, Command>... events){
        for(HashMap<String, Command> event : events){
            m_autoEventMap.putAll(event);
        }
    }

    public static void removeEvents(String... keys){
        for(String key : keys){
            m_autoEventMap.remove(key);
        }
    }

    public static HashMap<String, Command> getEvents(){
        return m_autoEventMap;
    }
}