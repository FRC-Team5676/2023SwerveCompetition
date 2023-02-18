package frc.robot.commands.accessories;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.xbox;
import frc.robot.subsystems.IntakeSubsystem;

public class OperateIntake extends CommandBase {
    private final IntakeSubsystem _intake;
    private final xbox _controller;

    public OperateIntake(xbox controller, IntakeSubsystem intake) {
        addRequirements(intake);
        _intake = intake;
        _controller = controller;
    }

    @Override
    public void execute() {
        double rate = _controller.getLeftStickY();
        _intake.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        _intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
