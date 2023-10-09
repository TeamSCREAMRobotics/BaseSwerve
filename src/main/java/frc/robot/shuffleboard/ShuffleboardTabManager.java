package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.shuffleboard.tabs.SwerveTab;

/**
 * Manages tabs for Shuffleboard.
 */
public class ShuffleboardTabManager extends SubsystemBase {

    private final ArrayList<ShuffleboardTabBase> m_tabs = new ArrayList<ShuffleboardTabBase>();

    public ShuffleboardTabManager() {
        if (ShuffleboardConstants.INCLUDE_DEBUG_TABS) {
            m_tabs.add(new SwerveTab(RobotContainer.getSwerve()));
        } else {
            /* Add code here */
        }

        for (ShuffleboardTabBase tab : m_tabs) {
            tab.createEntries();
        }
    }

    /**
     * Calls the periodic method for each Shuffleboard tab in the list of tabs.
     */
    public void periodic() {
        for (ShuffleboardTabBase tab : m_tabs) {
            tab.periodic();
    
    }
}
}