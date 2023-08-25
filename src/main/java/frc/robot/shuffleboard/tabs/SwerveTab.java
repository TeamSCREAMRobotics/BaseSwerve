package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Swerve;

public class SwerveTab extends ShuffleboardTabBase {

    private Swerve m_swerve;

    public SwerveTab(Swerve swerve) {
        m_swerve = swerve;
    }

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

        if (SwerveConstants.updateSwerveFromShuffleboard) {
            /* Add code here */
        }
    }

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

        if (SwerveConstants.updateSwerveFromShuffleboard) {
            /* Add code here */
        }
    }
}
