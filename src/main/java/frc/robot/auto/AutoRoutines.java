package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutoRoutines {

    public static Command exampleRoutine(Swerve swerve) {
        return swerve.followTrajectoryCommand(Paths.examplePath(), AutoEvents.exampleEvents(), true, true);
    }
}
