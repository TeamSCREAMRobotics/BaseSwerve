package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class Routines {

    /** 
     * @return A command that does nothing.
     */
    public static Command doNothing(){
        return new WaitCommand(15.0).withName("Do Nothing");
    }

    /** 
     * Returns a custom sequence of commands.
     * Uses Named Commands to trigger events along the path.
     * @return A custom sequence of commands.
     */
    public static Command exampleSequence(){
        Command routine = new SequentialCommandGroup(
            AutoBuilder.followPathWithEvents(Paths.getExamplePath())
        );
        
        return routine.withName("Example Sequence");
    }

    /** 
     * Returns a full auto created in the PathPlanner application.
     * Uses Named Commands to trigger events along the path.
     * @return A full autonomous command.
     */
    public static Command exampleAuto(){
        return new PathPlannerAuto("ExampleAuto").withName("Example Auto");
    }
}
