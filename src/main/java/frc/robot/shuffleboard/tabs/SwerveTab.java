
package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
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
     * Each module has three generic entries: Cancoder, Integrated, and Velocity.
     */
    private GenericEntry m_FLCancoder;
    private GenericEntry m_FLIntegrated;
    private GenericEntry m_FLVelocity;

    private GenericEntry m_FRCancoder;
    private GenericEntry m_FRIntegrated;
    private GenericEntry m_FRVelocity;

    private GenericEntry m_BLCancoder;
    private GenericEntry m_BLIntegrated;
    private GenericEntry m_BLVelocity;

    private GenericEntry m_BRCancoder;
    private GenericEntry m_BRIntegrated;
    private GenericEntry m_BRVelocity;

    private GenericEntry m_gyroYaw;

    private GenericEntry m_odometryX;
    private GenericEntry m_odometryY;
    private GenericEntry m_odometryYaw;

    /**
     * This method creates number entries for various sensors related to the Swerve subsystem.
     * These entries are used to display and update values on the Shuffleboard.
     * Set {@code SwerveConstants.updateSwerveFromShuffleboard} equal to true to add and/or update values
     */
    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Swerve");

        m_FLCancoder = createNumberEntry("FL Cancoder", 0);
        m_FLIntegrated = createNumberEntry("FL Integrated", 0);
        m_FLVelocity = createNumberEntry("FL Velocity", 0);

        m_FRCancoder = createNumberEntry("FR Cancoder", 0);
        m_FRIntegrated = createNumberEntry("FR Integrated", 0);
        m_FRVelocity = createNumberEntry("FR Velocity", 0);

        m_BLCancoder = createNumberEntry("BL Cancoder", 0);
        m_BLIntegrated = createNumberEntry("BL Integrated", 0);
        m_BLVelocity = createNumberEntry("BL Velocity", 0);

        m_BRCancoder = createNumberEntry("BR Cancoder", 0);
        m_BRIntegrated = createNumberEntry("BR Integrated", 0);
        m_BRVelocity = createNumberEntry("BR Velocity", 0);

        m_gyroYaw = createNumberEntry("Gyro Yaw", 0);

        m_odometryX = createNumberEntry("Odometry X", 0);
        m_odometryY = createNumberEntry("Odometry Y", 0);
        m_odometryYaw = createNumberEntry("Odometry Yaw", 0);

        if (SwerveConstants.UPDATE_SWERVE_FROM_SHUFFLEBOARD) {
            /* Add code here */
        }
    }

    /**
     * Updates the values of various Shuffleboard widgets with the current state of the swerve drive.
     * The Cancoder, Integrated, and Velocity widgets for each module are updated periodically.
     * Set {@code}updateSwerveFromShuffleboard{@code} to true to update values.
     */
    @Override
    public void periodic() {
        m_FLCancoder.setDouble(m_swerve.getModules()[0].getCanCoder().getDegrees());
        m_FLIntegrated.setDouble(m_swerve.getModules()[0].getPosition().angle.getDegrees());
        m_FLVelocity.setDouble(m_swerve.getModules()[0].getState().speedMetersPerSecond);

        m_FRCancoder.setDouble(m_swerve.getModules()[1].getCanCoder().getDegrees());
        m_FRIntegrated.setDouble(m_swerve.getModules()[1].getPosition().angle.getDegrees());
        m_FRVelocity.setDouble(m_swerve.getModules()[1].getState().speedMetersPerSecond);

        m_BLCancoder.setDouble(m_swerve.getModules()[2].getCanCoder().getDegrees());
        m_BLIntegrated.setDouble(m_swerve.getModules()[2].getPosition().angle.getDegrees());
        m_BLVelocity.setDouble(m_swerve.getModules()[2].getState().speedMetersPerSecond);

        m_BRCancoder.setDouble(m_swerve.getModules()[3].getCanCoder().getDegrees());
        m_BRIntegrated.setDouble(m_swerve.getModules()[3].getPosition().angle.getDegrees());
        m_BRVelocity.setDouble(m_swerve.getModules()[3].getState().speedMetersPerSecond);

        m_gyroYaw.setDouble(m_swerve.getYaw().getDegrees());

        m_odometryX.setDouble(m_swerve.getPose().getX());
        m_odometryY.setDouble(m_swerve.getPose().getY());
        m_odometryYaw.setDouble(m_swerve.getPose().getRotation().getDegrees());

        if (SwerveConstants.UPDATE_SWERVE_FROM_SHUFFLEBOARD) {
            /* Add code here */
        }
    }
}
