package frc.robot.subsystems.modules;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class LowerArmModule extends SubsystemBase {

    public final RelativeEncoder m_driveEncoder;
    public final CANSparkMax m_driveMotor;
    
    private final SparkMaxPIDController m_driveVelController;
    private final int VEL_SLOT = 1;

    public LowerArmModule(int driveMotorCanChannel, boolean driveMotorReversed) {
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

        m_driveVelController = m_driveMotor.getPIDController();
        m_driveVelController.setP(.01, VEL_SLOT);
        m_driveVelController.setD(0, VEL_SLOT);
        m_driveVelController.setI(0, VEL_SLOT);
        m_driveVelController.setIZone(1, VEL_SLOT);
    }

    public double getPosition() {
        return m_driveEncoder.getPosition();
    }

    public static double limitMotorCmd(double motorCmdIn) {
        return Math.max(Math.min(motorCmdIn, 1.0), -1.0);
    }

    public void setDriveBrakeMode(boolean on) {
        if (on) {
          m_driveMotor.setIdleMode(IdleMode.kBrake);
        } else {
          m_driveMotor.setIdleMode(IdleMode.kCoast);
        }
      }
}