
package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.ShuffleboardConstants;
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
     * Represents a set of entries for the Swerve subsystem.
     * Each module has three entries: CANcoder, Integrated, and Velocity readings.
     */
    private GenericEntry m_FLEncoder;
    private GenericEntry m_FLIntegrated;
    private GenericEntry m_FLVelocity;

    private GenericEntry m_FREncoder;
    private GenericEntry m_FRIntegrated;
    private GenericEntry m_FRVelocity;

    private GenericEntry m_BLEncoder;
    private GenericEntry m_BLIntegrated;
    private GenericEntry m_BLVelocity;

    private GenericEntry m_BREncoder;
    private GenericEntry m_BRIntegrated;
    private GenericEntry m_BRVelocity;

    private GenericEntry m_odometryX;
    private GenericEntry m_odometryY;
    private GenericEntry m_odometryYaw;

    private GenericEntry m_gyroYaw;

    private GenericEntry m_driveP;
    private GenericEntry m_driveI;
    private GenericEntry m_driveD;
    
    private GenericEntry m_angleP;
    private GenericEntry m_angleI;
    private GenericEntry m_angleD;

    /**
     * This method creates number entries for various sensors related to the Swerve subsystem.
     * These entries are used to display and update values on the Shuffleboard.
     * Set {@code SwerveConstants.updateSwerveFromShuffleboard} for entries that get values.
     */
    @Override
    public void createEntries() {
        m_tab = Shuffleboard.getTab("Swerve");

        m_FLEncoder = createNumberEntry("FL Encoder", 0, new EntryProperties(0, 0));
        m_FLIntegrated = createNumberEntry("FL Integrated", 0, new EntryProperties(1, 0));
        m_FLVelocity = createNumberEntry("FL Velocity", 0, new EntryProperties(2,0));

        m_FREncoder = createNumberEntry("FR Encoder", 0, new EntryProperties(0, 1));
        m_FRIntegrated = createNumberEntry("FR Integrated", 0, new EntryProperties(1, 1));
        m_FRVelocity = createNumberEntry("FR Velocity", 0, new EntryProperties(2, 1));

        m_BLEncoder = createNumberEntry("BL Encoder", 0, new EntryProperties(0, 2));
        m_BLIntegrated = createNumberEntry("BL Integrated", 0, new EntryProperties(1, 2));
        m_BLVelocity = createNumberEntry("BL Velocity", 0, new EntryProperties(2, 2));

        m_BREncoder = createNumberEntry("BR Encoder", 0, new EntryProperties(0, 3));
        m_BRIntegrated = createNumberEntry("BR Integrated", 0, new EntryProperties(1, 3));
        m_BRVelocity = createNumberEntry("BR Velocity", 0, new EntryProperties(2, 3));

        m_odometryX = createNumberEntry("Odometry X", 0, new EntryProperties(4, 0));
        m_odometryY = createNumberEntry("Odometry Y", 0, new EntryProperties(4, 1));
        m_odometryYaw = createNumberEntry("Odometry Yaw", 0, new EntryProperties(4, 2));

        m_gyroYaw = createEntry("Gyro Yaw", 0, new EntryProperties(6, 0, 3, 3, BuiltInWidgets.kGyro));

        if (ShuffleboardConstants.UPDATE_SWERVE) {
            
        }
    }

    

    /**
     * Updates the values of various Shuffleboard widgets with the current state of the swerve drive.
     * Set {@code}updateSwerveFromShuffleboard{@code} for getting values.
     */
    @Override
    public void periodic() {
        m_FLEncoder.setDouble(round(m_swerve.getModules()[0].getEncoder().getDegrees(), 3));
        m_FLIntegrated.setDouble(round(m_swerve.getModules()[0].getPosition().angle.getDegrees(), 3));
        m_FLVelocity.setDouble(round(m_swerve.getModules()[0].getState().speedMetersPerSecond, 3));

        m_FREncoder.setDouble(round(m_swerve.getModules()[1].getEncoder().getDegrees(), 3));
        m_FRIntegrated.setDouble(round(m_swerve.getModules()[1].getPosition().angle.getDegrees(), 3));
        m_FRVelocity.setDouble(round(m_swerve.getModules()[1].getState().speedMetersPerSecond, 3));

        m_BLEncoder.setDouble(round(m_swerve.getModules()[2].getEncoder().getDegrees(), 3));
        m_BLIntegrated.setDouble(round(m_swerve.getModules()[2].getPosition().angle.getDegrees(), 3));
        m_BLVelocity.setDouble(round(m_swerve.getModules()[2].getState().speedMetersPerSecond, 3));

        m_BREncoder.setDouble(round(m_swerve.getModules()[3].getEncoder().getDegrees(), 3));
        m_BRIntegrated.setDouble(round(m_swerve.getModules()[3].getPosition().angle.getDegrees(), 3));
        m_BRVelocity.setDouble(round(m_swerve.getModules()[3].getState().speedMetersPerSecond, 3));

        m_odometryX.setDouble(m_swerve.getPose().getX());
        m_odometryY.setDouble(m_swerve.getPose().getY());
        m_odometryYaw.setDouble(m_swerve.getPose().getRotation().getDegrees());

        if (ShuffleboardConstants.UPDATE_SWERVE) {
            /* Add code here */
        }
    }
}
