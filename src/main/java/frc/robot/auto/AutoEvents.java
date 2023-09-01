package frc.robot.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * A container class for PathPlanner events.
 * <p>
 * Auto events are used by PathPlanner to trigger events along a path.
 */
public class AutoEvents extends SequentialCommandGroup {

    public AutoEvents() {}

    public static HashMap<String, Command> doNothing() {
        return new HashMap<String, Command>();
    }

    public static HashMap<String, Command> exampleEvents() {
        HashMap<String, Command> exampleEvents = new HashMap<String, Command>();

        exampleEvents.put("Example", new InstantCommand());
        return exampleEvents;
    }
}
