package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class Paths {

    public static PathPlannerTrajectory examplePath() {
        return PathPlanner.loadPath("Example", new PathConstraints(0, 0));
    }
}
