package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class UpperArmSubsystem extends SubsystemBase {

  public double rotations;

  private final int m_upperArmCanId = 40;
  private final boolean m_upperArmMotorReversed = true;

  private final RelativeEncoder m_driveEncoder;
  private final CANSparkMax m_driveMotor;
  private final SparkMaxPIDController m_driveController;

  private final double minRotations = -2;
  private final double maxRotations = 39;

  public UpperArmSubsystem() {
    m_driveMotor = new CANSparkMax(m_upperArmCanId, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setInverted(m_upperArmMotorReversed);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveController = m_driveMotor.getPIDController();
    m_driveController.setP(0.01);
    m_driveController.setI(0);
    m_driveController.setD(0);
    m_driveController.setIZone(0);
    m_driveController.setFF(0);
    m_driveController.setOutputRange(-1, 1);

    ShuffleboardContent.initUpperArm(this);
  }

  @Override
  public void periodic() {
  }

  public void moveToPosition(double position) {
    setReferenceValue(position);
    setReferencePeriodic();
  }

  public void moveToMaxPosition() {
    setReferenceValue(36);
    setReferencePeriodic();
  }

  public void moveToMidPosition() {
    setReferenceValue(20);
    setReferencePeriodic();
  }

  public void moveToBottomPosition() {
    setReferenceValue(0);
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

  public void setReferenceValue(double rotation) {
    rotations = rotation;
  }

  public void setReferencePeriodic() {
    rotations = MathUtil.clamp(rotations, minRotations, maxRotations);
    m_driveController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

}