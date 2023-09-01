package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * A container class PathPlanner trajectories, including their respective max velocity and acceleration.
 */
public class Paths {

    public static PathPlannerTrajectory examplePath() {
        return PathPlanner.loadPath("Example", new PathConstraints(0, 0));
    }
}
