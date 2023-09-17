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

    private static RobotContainer m_container;

    public AutoRoutines(RobotContainer container){
        m_container = container;
    }

    public Command doNothing(){
        return new InstantCommand();
    }

    public Command exampleRoutine(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_container.getSwerve().resetPose(AutoTrajectories.getExampleTrajectory().getInitialHolonomicPose())),
            m_container.getSwerve().followTrajectoryCommand(AutoTrajectories.getExampleTrajectory(), true)
        );
    }
}
