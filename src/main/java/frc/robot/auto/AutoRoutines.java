package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.Tuple2;
import frc.robot.subsystems.Swerve;

/**
 * A utility class that contains predefined auto routines for use during the autonomous period.
 *
 */
public class AutoRoutines {

    /* Add additional subsystems here and in the constructor */
    private static Swerve m_swerve;

    private AutoEvents m_autoEvents;

    private Tuple2<PathPlannerTrajectory, HashMap<String, Command>> exampleSet = new Tuple2<>(AutoTrajectories.getExampleTrajectory(), AutoEvents.getEvents());

    public AutoRoutines(Swerve swerve){
        m_swerve = swerve;

        m_autoEvents = new AutoEvents(swerve);
    }

    public Command doNothing(){
        return new InstantCommand();
    }

    public Command exampleRoutine(){
        return m_swerve.followTrajectoryCommand(exampleSet, true, false);
    }
}
