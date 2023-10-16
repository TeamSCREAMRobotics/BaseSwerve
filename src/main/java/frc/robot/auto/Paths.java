package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;

/**
 * A class that contains PathPlanner paths.
 */
public class Paths {

    public static PathPlannerPath getExamplePath() {
        return PathPlannerPath.fromPathFile("ExamplePath");
    }
}
