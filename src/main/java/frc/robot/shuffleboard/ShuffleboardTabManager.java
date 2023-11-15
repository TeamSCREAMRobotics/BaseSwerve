package frc.robot.shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.SwerveTab;

/**
 * Manages tabs for Shuffleboard.
 */
public class ShuffleboardTabManager extends SubsystemBase {

    private static final ArrayList<ShuffleboardTabBase> m_tabs = new ArrayList<ShuffleboardTabBase>();

    /**
     * Adds predefined tabs to Shuffleboard.<p>
     * @param includeDebug Whether to include additional debug tabs.
     */
    public static void addTabs(boolean includeDebug){
        if (includeDebug) {
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