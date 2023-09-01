package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.SwerveTab;

/**
 * Manages the Shuffleboard tabs for the robot.
 */
public class ShuffleboardTabManager extends SubsystemBase {

    // Adds more tabs to use when debugging
    public final boolean mDebug = true;

    private final ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    public ShuffleboardTabManager(RobotContainer robotContainer) {
        if (mDebug) {
            mTabs.add(new SwerveTab(robotContainer.getSwerve()));
        } else {
            /* Add code here */
        }

        for (ShuffleboardTabBase tab : mTabs) {
            tab.createEntries();
        }
    }

    /**
     * Calls the periodic method for each Shuffleboard tab in the list of tabs.
     */
    public void periodic() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.periodic();
        }
    }
}