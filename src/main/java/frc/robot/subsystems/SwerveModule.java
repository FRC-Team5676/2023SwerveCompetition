// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Enums.ModulePosition;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.AngleUtils;
import frc.robot.utils.NeoConversions;
import frc.robot.utils.ShuffleboardContent;

public class SwerveModule extends SubsystemBase {

  public CANSparkMax m_driveMotor;
  public CANSparkMax m_turnMotor;
  public CANCoder m_turnCANcoder;
  public ModulePosition m_modulePosition;
  public int m_moduleNumber;
  public String[] m_modAbrev = { "_FL", "_FR", "_RL", "_RR" };
  public double m_desiredAngle;
  public double m_actualAngleDegrees;
  public double m_turningEncoderOffset;
  public boolean m_driveMotorConnected;
  public boolean m_turnMotorConnected;
  public boolean m_turnCoderConnected;

  private RelativeEncoder m_driveEncoder;
  private RelativeEncoder m_turnEncoder;
  private SparkMaxPIDController m_driveController;
  private SparkMaxPIDController m_turnController;
  private SwerveModuleState state;
  private double lastAngle;
  private double tolDegPerSec = .05;
  private double toleranceDeg = .25;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel       The channel of the drive motor.
   * @param turningMotorChannel     The channel of the turning motor.
   * @param driveEncoderChannels    The channels of the drive encoder.
   * @param turningCANCoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed    Whether the drive encoder is reversed.
   * @param turningEncoderReversed  Whether the turning encoder is reversed.
   * @param turningEncoderOffset    Offset of absolute encoder relative to front of robot
   */
  public SwerveModule(
      ModulePosition modulePosition,
      int driveMotorCanChannel,
      int turningMotorCanChannel,
      int cancoderCanChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double turningEncoderOffset) {

    configDriveMotor(driveMotorCanChannel, driveMotorReversed);
    configTurnMotor(turningMotorCanChannel, turningMotorReversed);
    configCanCoder(cancoderCanChannel, turningEncoderOffset);

    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();// gets module enum index

    checkCAN();
    resetAngleToAbsolute();

    ShuffleboardContent.initDriveShuffleboard(this);
    ShuffleboardContent.initTurnShuffleboard(this);
    ShuffleboardContent.initCANCoderShuffleboard(this);
    ShuffleboardContent.initBooleanShuffleboard(this);
    ShuffleboardContent.initCoderBooleanShuffleboard(this);
  }

  @Override
  public void periodic() {
    CANCoderFaults faults = new CANCoderFaults();
    m_turnCANcoder.getFaults(faults);
  }

  public SwerveModulePosition getPosition() {
    double position = m_turnEncoder.getPosition() - m_turningEncoderOffset;
    return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(position));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getHeadingDegrees()));
  }

  public ModulePosition getModulePosition() {
    return m_modulePosition;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    state = AngleUtils.optimize(desiredState, getHeadingRotation2d());

    /*
     * turn motor code
     * Prevent rotating module if speed is less then 1%. Prevents Jittering.
     */
    m_desiredAngle = (Math.abs(state.speedMetersPerSecond) <= (DriveConstants.kMaxAllowedVelMetersPerSec * 0.01))
        ? lastAngle
        : state.angle.getDegrees();
    lastAngle = m_desiredAngle;
    m_actualAngleDegrees = m_turnEncoder.getPosition();
    turnMotorMove(m_desiredAngle);

    /*
     * drive axis
     */
    if (isOpenLoop)
      m_driveMotor.set(state.speedMetersPerSecond
          / NeoConversions.maxMechMetersPerSec(ModuleConstants.kWheelCircumferenceMeters,
              ModuleConstants.kMk4iDriveGearRatio));
    else {
      m_driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }
  }

  public static double limitMotorCmd(double motorCmdIn) {
    return Math.max(Math.min(motorCmdIn, 1.0), -1.0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turnEncoder.setPosition(0);
  }

  public double getHeadingDegrees() {
      return m_turnEncoder.getPosition();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public void resetAngleToAbsolute() {
    double angle = m_turnCANcoder.getAbsolutePosition() - m_turningEncoderOffset;
    m_turnEncoder.setPosition(angle);
  }

  public double getTurnAngle() {
    return m_turnEncoder.getPosition();
  }

  public void turnMotorMove(double speed) {
    m_turnMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public void driveMotorMove(double speed) {
    m_driveMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getDriveCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  public double getTurnVelocity() {
    return m_turnEncoder.getVelocity();
  }

  public double getTurnPosition() {
    return m_turnEncoder.getPosition();
  }

  public double getTurnCurrent() {
    return m_turnMotor.getOutputCurrent();
  }

  public boolean turnInPosition(double targetAngle) {
    return Math.abs(targetAngle - getTurnAngle()) < toleranceDeg;
  }

  public boolean turnIsStopped() {
    return Math.abs(m_turnEncoder.getVelocity()) < tolDegPerSec;
  }

  public boolean checkCAN() {
    m_driveMotorConnected = m_driveMotor.getFirmwareVersion() != 0;
    m_turnMotorConnected = m_turnMotor.getFirmwareVersion() != 0;
    m_turnCoderConnected = m_turnCANcoder.getFirmwareVersion() > 0;

    return m_driveMotorConnected && m_turnMotorConnected && m_turnCoderConnected;
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }

  private void configDriveMotor(int driveMotorCanChannel, boolean driveMotorReversed) {
    // Motor setup
    m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
    m_driveMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);
    m_driveMotor.setInverted(driveMotorReversed);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // encoder setup
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingMetersPerEncRev);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingMetersPerEncRev / 60);

    // Controller setup
    m_driveController = m_driveMotor.getPIDController();
    m_driveController.setP(ModuleConstants.kDriveP);
    m_driveController.setI(ModuleConstants.kDriveI);
    m_driveController.setD(ModuleConstants.kDriveD);
    m_driveController.setIZone(ModuleConstants.kDriveIZone);
  }

  private void configTurnMotor(int turningMotorCanChannel, boolean turningMotorReversed) {
    // turning motor setup
    m_turnMotor = new CANSparkMax(turningMotorCanChannel, MotorType.kBrushless);
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
    m_turnMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);
    m_turnMotor.setInverted(turningMotorReversed);
    m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_turnMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
    m_turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // turning encoder setup
    // absolute encoder used to establish known wheel position on start position
    m_turnEncoder = m_turnMotor.getEncoder();
    m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);
    m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);

    m_turnController = m_turnMotor.getPIDController();
    m_turnController.setP(ModuleConstants.kTurnP);
    m_turnController.setI(ModuleConstants.kTurnI);
    m_turnController.setD(ModuleConstants.kTurnD);
    m_turnController.setIZone(ModuleConstants.kTurnIZone);
  }

  private void configCanCoder(int cancoderCanChannel, double turningEncoderOffset) {
    m_turnCANcoder = new CANCoder(cancoderCanChannel);
    m_turnCANcoder.configFactoryDefault();
    m_turnCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());
    m_turningEncoderOffset = turningEncoderOffset;
  }
}