package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

/*
 * Container class for SwerveModule constants.
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int CANcoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * 
     * @param driveMotorID The ID of the module's drive motor
     * @param angleMotorID The ID of the module's angle/steer motor
     * @param CANcoderID The ID of the module's CANcoder
     * @param angleOffset The offset to subtract from the CANcoder
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int CANcoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.CANcoderID = CANcoderID;
        this.angleOffset = angleOffset;
    }
}
