package frc.robot.auto;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A utility class that holds all events for use during the autonomous period.
 * Add events in RobotContainer.
 */
public class AutoEvents{
    
    private static final HashMap<String, Command> autoEventMap = new HashMap<String, Command>();
    
    public static void addEvent(String key, Command command){
        autoEventMap.put(key, command);
    }

    public static void addEventMap(HashMap<String, Command> eventMap){
        autoEventMap.putAll(eventMap);
    }

    public static void removeEvents(String... keys){
        for(String key : keys){
            autoEventMap.remove(key);
        }
    }

    public static HashMap<String, Command> getEvents(){
        return autoEventMap;
    }
}