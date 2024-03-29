// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Enums.ModulePosition;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final SwerveModule m_frontLeft = new SwerveModule(
            ModulePosition.FRONT_LEFT,
            DriveConstants.kFrontLeftDriveMotorCanId,
            DriveConstants.kFrontLeftTurnMotorCanId,
            DriveConstants.kFrontLeftTurnEncoderCanId,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftTurningMotorReversed,
            DriveConstants.kFrontLeftAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
            ModulePosition.FRONT_RIGHT,
            DriveConstants.kFrontRightDriveMotorCanId,
            DriveConstants.kFrontRightTurnMotorCanId,
            DriveConstants.kFrontRightTurnEncoderCanId,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightTurningMotorReversed,
            DriveConstants.kFrontRightAngularOffset);

    private final SwerveModule m_backLeft = new SwerveModule(
            ModulePosition.BACK_LEFT,
            DriveConstants.kBackLeftDriveMotorCanId,
            DriveConstants.kBackLeftTurnMotorCanId,
            DriveConstants.kBackLeftTurnEncoderCanId,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftTurningMotorReversed,
            DriveConstants.kBackLeftAngularOffset);

    private final SwerveModule m_backRight = new SwerveModule(
            ModulePosition.BACK_RIGHT,
            DriveConstants.kBackRightDriveMotorCanId,
            DriveConstants.kBackRightTurnMotorCanId,
            DriveConstants.kBackRightTurnEncoderCanId,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightTurningMotorReversed,
            DriveConstants.kBackRightAngularOffset);

    // The gyro sensor
    private final AHRS m_gyro;
    private final SwerveDriveOdometry m_odometry;

    public boolean m_fieldRelative = Constants.CustomConstants.fieldRelative;
    public Boolean m_swerveHighSpeedMode;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        m_swerveHighSpeedMode = true;

        m_gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();

        // Odometry class for tracking robot pose
        m_odometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        SmartDashboard.putNumber("Gyroscope Angle", m_gyro.getAngle());

        m_odometry.update(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the currently-estimated yaw of the robot.
     *
     * @return The yaw.
     */
    public double getYaw() {
        return m_gyro.getYaw();
    }

    /**
     * Returns the currently-estimated roll of the robot.
     *
     * @return The roll.
     */
    public double getRoll() {
        return m_gyro.getRoll();
    }

    /**
     * Returns the currently-estimated pitch of the robot.
     *
     * @return The pitch.
     */
    public double getPitch() {
        return m_gyro.getPitch();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(m_gyro.getAngle()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double throttle, double strafe, double rotation, boolean isOpenLoop) {

        throttle *= DriveConstants.kMaxSpeedMetersPerSecond;
        strafe *= DriveConstants.kMaxSpeedMetersPerSecond;
        rotation *= DriveConstants.kMaxRotationRadiansPerSecond;

        SmartDashboard.putNumber("Rotn1", rotation);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                m_fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                throttle, strafe, rotation, Rotation2d.fromDegrees(-m_gyro.getYaw()))
                        : new ChassisSpeeds(throttle, strafe, rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0], isOpenLoop);
        m_frontRight.setDesiredState(swerveModuleStates[1], isOpenLoop);
        m_backLeft.setDesiredState(swerveModuleStates[2], isOpenLoop);
        m_backRight.setDesiredState(swerveModuleStates[3], isOpenLoop);
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public void setX() {
        boolean isOpenLoop = true;
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), isOpenLoop);
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), isOpenLoop);
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), isOpenLoop);
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), isOpenLoop);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0], isOpenLoop);
        m_frontRight.setDesiredState(desiredStates[1], isOpenLoop);
        m_backLeft.setDesiredState(desiredStates[2], isOpenLoop);
        m_backRight.setDesiredState(desiredStates[3], isOpenLoop);
    }

    public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, true);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backLeft.resetEncoders();
        m_backRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroGyro() {
        m_gyro.zeroYaw();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void toggleSwerveMode() {
        m_swerveHighSpeedMode = !m_swerveHighSpeedMode;
    }

    public void toggleFieldRelative() {
        m_fieldRelative = !m_fieldRelative;
    }

  // Stop all module movement
  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }
}
