package frc.robot.subsystems.modules;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.ShuffleboardContent;

public class UpperArmModule extends SubsystemBase {

  public double rotations;

  private final RelativeEncoder m_driveEncoder;
  private final CANSparkMax m_driveMotor;
  private final SparkMaxPIDController m_driveController;

  private final double minRotations = 360 * 0;
  private final double maxRotations = 360 * 6;

  public UpperArmModule(int driveMotorCanChannel, boolean driveMotorReversed) {
    // Drive Motor setup
    m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_driveMotor.enableVoltageCompensation(DriveConstants.kVoltCompensation);
    m_driveMotor.setInverted(driveMotorReversed);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    m_driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // drive encoder setup
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncRPMperMPS);

    m_driveController = m_driveMotor.getPIDController();
    m_driveController.setP(0.1);
    m_driveController.setI(1e-4);
    m_driveController.setD(1);
    m_driveController.setIZone(0);
    m_driveController.setFF(0);
    m_driveController.setOutputRange(-1, 1);

    ShuffleboardContent.initUpperArm(this);
  }

  public double getMinRotations() {
    return Math.toRadians(minRotations);
  }

  public double getMaxRotations() {
    return Math.toRadians(maxRotations);
  }

  public double getPosition() {
    return m_driveEncoder.getPosition();
  }

  public void moveArm(double throttle) {
    rotations += throttle;
    rotations = MathUtil.clamp(rotations, getMinRotations(), getMaxRotations());
    // m_driveMotor.set(throttle);
  }

  public void stop() {
    m_driveMotor.set(0);
  }

  public void setReferenceValue(double rotation) {
    rotations = rotation;
  }

  public void setReferencePeriodic() {
    m_driveController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

}