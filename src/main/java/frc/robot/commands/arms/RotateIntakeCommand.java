package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.xbox;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateIntakeCommand extends CommandBase {

    private final IntakeSubsystem m_controlArm;
    private final xbox m_controller;

    /** Driver control */
    public RotateIntakeCommand(IntakeSubsystem controlArm, xbox controller) {
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
        double throttle = m_controller.getRightTrigger() - m_controller.getLeftTrigger();
        m_controlArm.rotateIntake(throttle * 0.10);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //m_controlArm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
