package frc.robot.commands.accessories;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.xbox;
import frc.robot.subsystems.ControlArmSubsystem;

public class OperateControlArm extends CommandBase {
    private final ControlArmSubsystem _controlArm;
    private final xbox _controller;

    public OperateControlArm(xbox controller, ControlArmSubsystem controlArm) {
        addRequirements(controlArm);
        _controlArm = controlArm;
        _controller = controller;
    }

    @Override
    public void execute() {
        double rate = _controller.getLeftStickY();
        _controlArm.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        _controlArm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
