package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.xbox;
import frc.robot.subsystems.ControlArmSubsystem;

public class MoveArms extends CommandBase {

    private final ControlArmSubsystem m_controlArm;
    private final xbox m_controller;

    /** Driver control */
    public MoveArms(ControlArmSubsystem controlArm, xbox controller) {
        m_controlArm = controlArm;
        m_controller = controller;

        addRequirements(controlArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_controlArm.driveLowerArm(m_controller.getRightStickY());
        m_controlArm.driveUpperArm(m_controller.getLeftStickY());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //m_controlArm.stopLowerArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
