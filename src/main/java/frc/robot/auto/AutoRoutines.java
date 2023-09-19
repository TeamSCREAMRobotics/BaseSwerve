package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/**
 * A utility class that contains predefined auto routines for use during the autonomous period.
 */
public class AutoRoutines extends CommandBase{

    public static Command doNothing(){
        return new InstantCommand().withName("Do Nothing");
    }

    public static Command exampleRoutine(){
        RobotContainer.getSwerve().resetPoseFromTraj(AutoTrajectories.getExampleTrajectory());

        Command routine = new SequentialCommandGroup(
            RobotContainer.getSwerve().followTrajectoryCommand(AutoTrajectories.getExampleTrajectory(), true)
        );
        
        return routine.withName("Example Routine");
    }
}
