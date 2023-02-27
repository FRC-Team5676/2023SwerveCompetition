package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.modules.LowerArmModule;
import frc.robot.subsystems.modules.UpperArmModule;

public class ControlArmSubsystem extends SubsystemBase {

    private final int m_upperArmCanId = 40;
    private final boolean m_upperArmMotorReversed = true;
    private final int m_lowerArmCanId = 6;
    private final boolean m_lowerArmMotorReversed = true;
    private final UpperArmModule m_upperArm = new UpperArmModule(m_upperArmCanId, m_upperArmMotorReversed);
    private final LowerArmModule m_lowerArm = new LowerArmModule(m_lowerArmCanId, m_lowerArmMotorReversed);

    public void driveUpperArm(double throttle) {
        m_upperArm.moveArm(throttle);
    }

    public void stopUpperArm() {
        m_upperArm.moveArm(0);
    }

    public void driveLowerArm(double throttle) {
        m_lowerArm.moveArm(throttle);
    }

    public void stopLowerArm() {
        m_lowerArm.moveArm(0);
    }

    @Override
    public void periodic() {
        // TODO: Add SmartDashboard stuff
    }
}
