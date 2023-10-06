package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * A utility class that contains PathPlanner trajectories, including their respective max velocity and acceleration.
 */
public class AutoTrajectories {

    public static PathPlannerTrajectory getExampleTrajectory() {
        return PathPlanner.loadPath("ExamplePath", new PathConstraints(1, 1));
    }
}
