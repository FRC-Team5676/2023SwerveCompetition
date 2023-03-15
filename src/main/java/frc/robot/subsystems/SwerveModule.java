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

import edu.wpi.first.math.controller.PIDController;
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
  private PIDController m_turnPidController;
  private SwerveModuleState m_state;
  private double m_toleranceDegPerSec = .05;
  private double m_toleranceDeg = .25;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel       The channel of the drive motor
   * @param turningMotorChannel     The channel of the turning motor
   * @param driveEncoderChannels    The channels of the drive encoder
   * @param turningCANCoderChannels The channels of the turning encoder
   * @param driveEncoderReversed    Whether the drive encoder is reversed
   * @param turningEncoderReversed  Whether the turning encoder is reversed
   * @param turningEncoderOffset    Offset of abs encoder relative to front
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
    double position = getTurnPosition() - m_turningEncoderOffset;
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(position));
  }

  public ModulePosition getModulePosition() {
    return m_modulePosition;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (Math.abs(m_state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    m_state = AngleUtils.optimize(desiredState, getState().angle);
    m_driveMotor.set(m_state.speedMetersPerSecond / DriveConstants.kMaxAllowedVelMetersPerSec);
    m_turnMotor.set(m_turnPidController.calculate(getTurnPosition(), m_state.angle.getDegrees()));

    m_actualAngleDegrees = getTurnPosition();
  }

  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turnEncoder.setPosition(0);
  }

  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getDriveCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  public double getTurnPosition() {
    return m_turnEncoder.getPosition();
  }

  public double getTurnVelocity() {
    return m_turnEncoder.getVelocity();
  }

  public double getTurnCurrent() {
    return m_turnMotor.getOutputCurrent();
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turnMotor.set(0);
  }

  private SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  private void resetAngleToAbsolute() {
    double angle = m_turnCANcoder.getAbsolutePosition() - m_turningEncoderOffset;
    m_turnEncoder.setPosition(angle);
  }

  private boolean checkCAN() {
    m_driveMotorConnected = m_driveMotor.getFirmwareVersion() != 0;
    m_turnMotorConnected = m_turnMotor.getFirmwareVersion() != 0;
    m_turnCoderConnected = m_turnCANcoder.getFirmwareVersion() > 0;

    return m_driveMotorConnected && m_turnMotorConnected && m_turnCoderConnected;
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
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRpm2Mps);

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
    m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
    m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRpm2Rps);

    m_turnPidController = new PIDController(ModuleConstants.kTurnP,
        ModuleConstants.kTurnI,
        ModuleConstants.kTurnD);
    m_turnPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private void configCanCoder(int cancoderCanChannel, double turningEncoderOffset) {
    m_turnCANcoder = new CANCoder(cancoderCanChannel);
    m_turnCANcoder.configFactoryDefault();
    m_turnCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());
    m_turningEncoderOffset = turningEncoderOffset;
  }
}