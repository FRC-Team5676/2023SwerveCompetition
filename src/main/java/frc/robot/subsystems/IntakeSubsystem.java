package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class IntakeSubsystem extends SubsystemBase {

  public double rotations;

  private final int m_intakeArmCanId = 30;
  private final boolean m_intakeArmMotorReversed = true;

  private final RelativeEncoder m_driveEncoder;
  private final CANSparkMax m_driveMotor;
  private final SparkMaxPIDController m_driveController;

  private final double minRotations = 360 * 0;
  private final double maxRotations = 360 * 0.5;

  public IntakeSubsystem() {
    // Drive Motor setup
    m_driveMotor = new CANSparkMax(m_intakeArmCanId, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setInverted(m_intakeArmMotorReversed);
    m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // drive encoder setup
    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveController = m_driveMotor.getPIDController();
    m_driveController.setP(0.1);
    m_driveController.setI(1e-4);
    m_driveController.setD(1);
    m_driveController.setIZone(0);
    m_driveController.setFF(0);
    m_driveController.setOutputRange(-1, 1);

    ShuffleboardContent.initIntakeArm(this);
  }

  @Override
  public void periodic() {
    setReferencePeriodic();
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

  public void setIntakePosition(double position) {
    setReferenceValue(position);
  }

  public void rotateIntake(double throttle) {
    m_driveMotor.set(throttle);
  }

  public void stop() {
    m_driveMotor.set(0);
  }

  public void setReferencePeriodic() {
    m_driveController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  public void setReferenceValue(double rotation) {
    rotations = rotation;
  }

  public void moveIntakeToPosition(double position) {
    setReferenceValue(position);
    setReferencePeriodic();
  }
}