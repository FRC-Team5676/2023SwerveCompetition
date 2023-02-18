package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HighConePlace extends CommandBase {

    private final ControlArmSubsystem _controlArm;
    private final IntakeSubsystem _intake;

    public HighConePlace(
            ControlArmSubsystem controlArm,
            IntakeSubsystem intake) {
        addRequirements(controlArm, intake);
        _controlArm = controlArm;
        _intake = intake;
    }

    @Override
    public void execute() {
        _controlArm.setHeight(Constants.Position.Cones.HighNodePosition.ControlArmPosition);
        _intake.setRotation(Constants.Position.Cones.HighNodePosition.IntakePosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
