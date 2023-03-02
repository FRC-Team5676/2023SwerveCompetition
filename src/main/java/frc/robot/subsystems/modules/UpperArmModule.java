package frc.robot.subsystems.modules;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class UpperArmModule extends SubsystemBase {

  public double rotations;

  private final RelativeEncoder m_driveEncoder;
  private final CANSparkMax m_driveMotor;
  private final SparkMaxPIDController m_driveController;

  private final double minRotations = 360 * 0;
  private final double maxRotations = 360 * 6;

  public UpperArmModule(int driveMotorCanChannel, boolean driveMotorReversed) {
    m_driveMotor = new CANSparkMax(driveMotorCanChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setInverted(driveMotorReversed);
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
    //setReferencePeriodic();
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

  public void driveArm(double throttle) {
    m_driveMotor.set(throttle);
  }

  public void setReferenceValue(double rotation) {
    rotations = rotation;
  }

  public void setReferencePeriodic() {
    m_driveController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

}