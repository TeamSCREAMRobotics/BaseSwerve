package frc.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.AutoEvents;
import frc.robot.subsystems.Swerve;

public class FollowTrajectoryCommand extends SequentialCommandGroup
{

  /**
     * Generates a command that follows the given trajectory.
     * Will automatically trigger events associated with that trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @param mirrorWithAlliance If the trajectory should be flipped according to alliance color.
     * @return The command that follows the trajectory and triggers associated events along it.
     */
  public FollowTrajectoryCommand(Swerve swerve, PathPlannerTrajectory trajectory, boolean mirrorWithAlliance)
  {
    addRequirements(swerve);

    addCommands(
        new FollowPathWithEvents(
            new PPSwerveControllerCommand(
                trajectory, 
                swerve::getPose, // Pose supplier
                SwerveConstants.SWERVE_KINEMATICS, // SwerveDriveKinematics
                SwerveConstants.PATH_TRANSLATION_CONTROLLER, // X controller
                SwerveConstants.PATH_TRANSLATION_CONTROLLER, // Y controller 
                SwerveConstants.PATH_ROTATION_CONTROLLER, // Rotation controller
                swerve::setModuleStates, // Module states consumer
                mirrorWithAlliance, // If the path should be mirrored depending on alliance color
                swerve // Requires this swerve subsystem
            ), 
            trajectory.getMarkers(), 
            AutoEvents.getEvents())
    );
  }
}