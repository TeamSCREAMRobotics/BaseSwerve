package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.FollowTrajectoryCommand;
import frc.robot.subsystems.Swerve;

/**
 * A utility class that contains predefined auto routines for use during the autonomous period.
 * Reference routines in RobotContainer.
 */
public class AutoRoutines{

    public static Command doNothing(){
        return new InstantCommand().withName("Do Nothing");
    }

    public static Command exampleRoutine(){
        Swerve swerve = RobotContainer.getSwerve();
        Command routine = new SequentialCommandGroup(
            new InstantCommand(() -> swerve.resetPose(AutoTrajectories.getExampleTrajectory())),
            new FollowTrajectoryCommand(swerve, AutoTrajectories.getExampleTrajectory(), true)
        );
        
        return routine.withName("Example Routine");
    }
}
