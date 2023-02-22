package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StowPosition extends CommandBase {

    private final ControlArmSubsystem _controlArm;
    private final IntakeSubsystem _intake;
    
    public StowPosition(
        ControlArmSubsystem controlArm,
        IntakeSubsystem intake) {
        addRequirements(controlArm, intake);
        _controlArm = controlArm;
        _intake = intake;
    }

    @Override
    public void execute() {
        //_controlArm.setHeight(Constants.Position.StowPosition.ControlArmUpperPosition);
        _intake.setRotation(Constants.Position.StowPosition.IntakePosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
