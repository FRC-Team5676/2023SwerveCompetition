package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.modules.LowerArmModule;
import frc.robot.subsystems.modules.UpperArmModule;

public class ControlArmSubsystem extends SubsystemBase {

    private UpperArmModule m_upperArm;
    private LowerArmModule m_lowerArm;

    private final int m_upperArmCanId = 40;
    private final boolean m_upperArmMotorReversed = true;
    private final int m_lowerArmCanId = 6;
    private final boolean m_lowerArmMotorReversed = true;

    public ControlArmSubsystem() {
        UpperArmModule m_upperArm = new UpperArmModule(m_upperArmCanId, m_upperArmMotorReversed);
        LowerArmModule m_lowerArm = new LowerArmModule(m_lowerArmCanId, m_lowerArmMotorReversed);
    }

    public void driveUpperArm(double throttle) {
        m_upperArm.m_driveMotor.set(throttle);
    }

    public void stopUpperArm() {
        m_upperArm.m_driveMotor.set(0);
    }

    public void driveLowerArm(double throttle) {
        m_lowerArm.m_driveMotor.set(throttle);
    }

    public void stopLowerArm() {
        m_lowerArm.m_driveMotor.set(0);
    }

    @Override
    public void periodic() {
        // TODO: Add SmartDashboard stuff
    }
}
