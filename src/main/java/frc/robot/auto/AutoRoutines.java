package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

/**
 * A container class for auto routines for use during the autonomous period.
 */
public class AutoRoutines {

    public static Command doNothing(){
        return new InstantCommand();
    }

    public static Command exampleRoutine(Swerve swerve) {
        return swerve.followTrajectoryCommand(Paths.examplePath(), AutoEvents.exampleEvents(), true, false);
    }
}
