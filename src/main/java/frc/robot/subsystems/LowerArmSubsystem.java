package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class LowerArmSubsystem extends SubsystemBase {

  public double rotations;

  private final int m_lowerArmCanId = 41;
  private final boolean m_lowerArmMotorReversed = true;

  private final RelativeEncoder m_driveEncoder;
  private final CANSparkMax m_driveMotor;
  private final SparkMaxPIDController m_driveController;

  private final double minRotations = -20;
  private final double maxRotations = 90;

  public LowerArmSubsystem() {
    // Drive Motor setup
    m_driveMotor = new CANSparkMax(m_lowerArmCanId, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setInverted(m_lowerArmMotorReversed);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // drive encoder setup
    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveController = m_driveMotor.getPIDController();
    m_driveController.setP(0.01);
    m_driveController.setI(0);
    m_driveController.setD(0);
    m_driveController.setOutputRange(-1, 1);

    ShuffleboardContent.initLowerArm(this);
  }

  @Override
  public void periodic() {
  }

  public void moveToPosition(double position) {
    setReferenceValue(position);
    setReferencePeriodic();
  }

  public void moveToFarPosition() {
    setReferenceValue(maxRotations);
    setReferencePeriodic();
  }

  public void moveToMidPosition() {
    setReferenceValue(22);
    setReferencePeriodic();
  }

  public void moveToBackPosition() {
    setReferenceValue(-20);
    setReferencePeriodic();
  }

  public double getMinRotations() {
    return minRotations;
  }

  public double getMaxRotations() {
    return maxRotations;
  }

  public double getPosition() {
    return m_driveEncoder.getPosition();
  }

  public void driveArm(double throttle) {
    throttle = -throttle;
    rotations += throttle;
    setReferencePeriodic();
  }

  public void stop() {
    m_driveMotor.set(0);
  }

  public void setReferenceValue(double rotation) {
    rotations = rotation;
  }

  public void setReferencePeriodic() {
    rotations = MathUtil.clamp(rotations, minRotations, maxRotations);
    m_driveController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }
}