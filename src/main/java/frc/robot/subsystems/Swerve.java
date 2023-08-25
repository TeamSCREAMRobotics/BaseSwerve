package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Mod0;
import frc.robot.Constants.SwerveConstants.Mod1;
import frc.robot.Constants.SwerveConstants.Mod2;
import frc.robot.Constants.SwerveConstants.Mod3;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.HashMap;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator m_swervePoseEstimator;
    private SwerveModule[] m_swerveModules;
    private Pigeon2 m_gyro;

    public Swerve() {
        m_gyro = new Pigeon2(Ports.pigeonID, Ports.canivoreBusName);
        m_gyro.configFactoryDefault();
        zeroGyro();

        m_swerveModules = new SwerveModule[] {
                new SwerveModule("FL", 0, Mod0.constants),
                new SwerveModule("FR", 1, Mod1.constants),
                new SwerveModule("BL", 2, Mod2.constants),
                new SwerveModule("BR", 3, Mod3.constants)
        };
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getYaw(),
                getModulePositions(), new Pose2d());
    }

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : m_swerveModules) {
            mod.set(swerveModuleStates[mod.getModuleNumber()], false);
        }
    }

    public void updatePoseWithVision(Pose2d pose) {
        m_swervePoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }

    public void resetPose(Pose2d pose) {
        m_swervePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return m_swervePoseEstimator.getEstimatedPosition();
    }

    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveModules) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
                : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    @Override
    public void periodic() {
        m_swervePoseEstimator.update(getYaw(), getModulePositions());
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, HashMap<String, Command> eventMap,
            boolean isFirstPath, boolean useAllianceColor) {
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                this::getPose,
                this::resetPose,
                SwerveConstants.pathTranslationConstants,
                SwerveConstants.pathRotationConstants,
                this::drive,
                eventMap,
                useAllianceColor,
                this);

        return autoBuilder.followPathWithEvents(traj);
    }
}