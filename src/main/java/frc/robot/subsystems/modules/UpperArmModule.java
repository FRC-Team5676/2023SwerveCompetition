package frc.robot.subsystems.modules;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.ShuffleboardContent;

public class UpperArmModule extends SubsystemBase {

  public double rotations;

  private final RelativeEncoder m_driveEncoder;
  private final SparkMaxPIDController m_driveController;
  private final CANSparkMax m_driveMotor;
  private final int VEL_SLOT = 1;

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
    m_driveController.setP(.01, VEL_SLOT);
    m_driveController.setD(0, VEL_SLOT);
    m_driveController.setI(0, VEL_SLOT);
    m_driveController.setIZone(1, VEL_SLOT);

    ShuffleboardContent.initUpperArm(this);
  }

  public double getMinRotations() {
    return 0;
  }

  public double getMaxRotations() {
    return Math.toRadians(360 * 6);
  }

  public double getPosition() {
    return m_driveEncoder.getPosition();
  }

  public void moveArm(double throttle) {
    //rotations = MathUtil.clamp(rotations, getMinRotations(), getMaxRotations());
    //rotations += throttle;
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
}