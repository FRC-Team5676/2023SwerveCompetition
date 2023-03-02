package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.modules.LowerArmModule;
import frc.robot.subsystems.modules.UpperArmModule;

public class ControlArmSubsystem extends SubsystemBase {

    private final int m_upperArmCanId = 40;
    private final boolean m_upperArmMotorReversed = false;
    private final int m_lowerArmCanId = 41;
    private final boolean m_lowerArmMotorReversed = false;

    private final UpperArmModule m_upperArm = new UpperArmModule(m_upperArmCanId, m_upperArmMotorReversed);
    private final LowerArmModule m_lowerArm = new LowerArmModule(m_lowerArmCanId, m_lowerArmMotorReversed);

    @Override
    public void periodic() {
        m_upperArm.setReferencePeriodic();
        m_lowerArm.setReferencePeriodic();
    }
  
     // Upper Arm
    public void driveUpperArm(double throttle) {
        m_upperArm.driveArm(throttle);
    }

    public void moveUpperArmToPosition(double position) {
        m_upperArm.setReferenceValue(position);
        m_upperArm.setReferencePeriodic();
    }

    // Lower Arm
    public void driveLowerArm(double throttle) {
        m_lowerArm.driveArm(throttle);
    }

    public void moveLowerArmToPosition(double position) {
        m_lowerArm.setReferenceValue(position);
        m_lowerArm.setReferencePeriodic();
    }
}
