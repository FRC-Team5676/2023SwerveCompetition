package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class IntakeSubsystem extends SubsystemBase {

  public double rotations;

  private final int m_leftintakeArmCanId = 21;
  private final boolean m_leftintakeArmMotorReversed = true;

  private final RelativeEncoder m_leftdriveEncoder;
  private final CANSparkMax m_leftdriveMotor;
  private final SparkMaxPIDController m_leftdriveController;

  
  private final double minRotations = 0;
  private final double maxRotations = 9.23;

  public IntakeSubsystem() {
    // Drive Motor setup
    m_leftdriveMotor = new CANSparkMax(m_leftintakeArmCanId, MotorType.kBrushless);
    m_leftdriveMotor.restoreFactoryDefaults();
    m_leftdriveMotor.setInverted(m_leftintakeArmMotorReversed);
    m_leftdriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    
    // drive encoder setup
    m_leftdriveEncoder = m_leftdriveMotor.getEncoder();

    m_leftdriveController = m_leftdriveMotor.getPIDController();
    m_leftdriveController.setP(0.01);
    m_leftdriveController.setI(0);
    m_leftdriveController.setD(0);
    m_leftdriveController.setIZone(0);
    m_leftdriveController.setFF(0);
    m_leftdriveController.setOutputRange(-1, 1);

   

    ShuffleboardContent.initIntakeArm(this);
  }

  @Override
  public void periodic() {
    //setReferencePeriodic();
  }

  public double getMinRotations() {
    return minRotations;
  }

  public double getMaxRotations() {
    return maxRotations;
  }

  public double getPosition() {
    return m_leftdriveEncoder.getPosition();
  }

  public void setIntakePosition(double position) {
    setReferenceValue(position);
  }

  public void rotateIntake(double throttle) {
    m_leftdriveMotor.set(throttle);
  }

  public void stop() {
    m_leftdriveMotor.set(0);
  }

  public void setReferencePeriodic() {
    m_leftdriveController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  
  public void setReferenceValue(double rotation) {
    rotations = rotation;
  }

  public void moveIntakeToPosition(double position) {
    setReferenceValue(position);
    setReferencePeriodic();
  }
}