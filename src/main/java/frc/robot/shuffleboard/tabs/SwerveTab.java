
package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Swerve;

public class SwerveTab extends ShuffleboardTabBase {

    private Swerve m_swerve;

    /**
     * A Shuffleboard tab for displaying and updating Swerve drive subsystem data.
     */
    public SwerveTab(Swerve swerve) {
        m_swerve = swerve;
    }
    
    /**
     * Represents a set of generic entries for the Swerve subsystem.
     * Each module has three generic entries: CANCoder, Integrated, and Velocity.
     */
    private GenericEntry m_FLCANCoder;
    private GenericEntry m_FLIntegrated;
    private GenericEntry m_FLVelocity;

    private GenericEntry m_FRCANCoder;
    private GenericEntry m_FRIntegrated;
    private GenericEntry m_FRVelocity;

    private GenericEntry m_BLCANCoder;
    private GenericEntry m_BLIntegrated;
    private GenericEntry m_BLVelocity;

    private GenericEntry m_BRCANCoder;
    private GenericEntry m_BRIntegrated;
    private GenericEntry m_BRVelocity;

    private GenericEntry m_odometryX;
    private GenericEntry m_odometryY;
    private GenericEntry m_odometryYaw;

    private GenericEntry m_gyroYaw;
    
    /**
     * This method creates number entries for various sensors related to the Swerve subsystem.
     * These entries are used to display and update values on the Shuffleboard.
     * Set {@code SwerveConstants.updateSwerveFromShuffleboard} equal to true to add and/or update values
     */
    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Swerve");

        m_FLCANCoder = createNumberEntry("FL CANCoder", 0, new EntryProperties(0, 0));
        m_FLIntegrated = createNumberEntry("FL Integrated", 0, new EntryProperties(1, 0));
        m_FLVelocity = createNumberEntry("FL Velocity", 0, new EntryProperties(2,0));

        m_FRCANCoder = createNumberEntry("FR CANCoder", 0, new EntryProperties(0, 1));
        m_FRIntegrated = createNumberEntry("FR Integrated", 0, new EntryProperties(1, 1));
        m_FRVelocity = createNumberEntry("FR Velocity", 0, new EntryProperties(2, 1));

        m_BLCANCoder = createNumberEntry("BL CANCoder", 0, new EntryProperties(0, 2));
        m_BLIntegrated = createNumberEntry("BL Integrated", 0, new EntryProperties(1, 2));
        m_BLVelocity = createNumberEntry("BL Velocity", 0, new EntryProperties(2, 2));

        m_BRCANCoder = createNumberEntry("BR CANCoder", 0, new EntryProperties(0, 3));
        m_BRIntegrated = createNumberEntry("BR Integrated", 0, new EntryProperties(1, 3));
        m_BRVelocity = createNumberEntry("BR Velocity", 0, new EntryProperties(2, 3));

        m_odometryX = createNumberEntry("Odometry X", 0, new EntryProperties(4, 0));
        m_odometryY = createNumberEntry("Odometry Y", 0, new EntryProperties(4, 1));
        m_odometryYaw = createNumberEntry("Odometry Yaw", 0, new EntryProperties(4, 2));

        m_gyroYaw = createEntry("Gyro Yaw", 0, BuiltInWidgets.kGyro, new EntryProperties(6, 0, 2, 2));

        if (SwerveConstants.UPDATE_SWERVE_FROM_SHUFFLEBOARD) {
            /* Add code here */
        }
    }

    

    /**
     * Updates the values of various Shuffleboard widgets with the current state of the swerve drive.
     * The CANCoder, Integrated, and Velocity widgets for each module are updated periodically.
     * Set {@code}updateSwerveFromShuffleboard{@code} to true to update values.
     */
    @Override
    public void periodic() {
        m_FLCANCoder.setDouble(round(m_swerve.getModules()[0].getCanCoder().getDegrees(), 3));
        m_FLIntegrated.setDouble(round(m_swerve.getModules()[0].getPosition().angle.getDegrees(), 3));
        m_FLVelocity.setDouble(round(m_swerve.getModules()[0].getState().speedMetersPerSecond, 3));

        m_FRCANCoder.setDouble(round(m_swerve.getModules()[1].getCanCoder().getDegrees(), 3));
        m_FRIntegrated.setDouble(round(m_swerve.getModules()[1].getPosition().angle.getDegrees(), 3));
        m_FRVelocity.setDouble(round(m_swerve.getModules()[1].getState().speedMetersPerSecond, 3));

        m_BLCANCoder.setDouble(round(m_swerve.getModules()[2].getCanCoder().getDegrees(), 3));
        m_BLIntegrated.setDouble(round(m_swerve.getModules()[2].getPosition().angle.getDegrees(), 3));
        m_BLVelocity.setDouble(round(m_swerve.getModules()[2].getState().speedMetersPerSecond, 3));

        m_BRCANCoder.setDouble(round(m_swerve.getModules()[3].getCanCoder().getDegrees(), 3));
        m_BRIntegrated.setDouble(round(m_swerve.getModules()[3].getPosition().angle.getDegrees(), 3));
        m_BRVelocity.setDouble(round(m_swerve.getModules()[3].getState().speedMetersPerSecond, 3));

        m_odometryX.setDouble(m_swerve.getPose().getX());
        m_odometryY.setDouble(m_swerve.getPose().getY());
        m_odometryYaw.setDouble(m_swerve.getPose().getRotation().getDegrees());

        m_gyroYaw.setDouble(m_swerve.getYaw().getDegrees());

        if (SwerveConstants.UPDATE_SWERVE_FROM_SHUFFLEBOARD) {
            /* Add code here */
        }
    }
}
