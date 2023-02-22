package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StowPosition extends CommandBase {

    private final ControlArmSubsystem m_controlArm;
    private final IntakeSubsystem m_intake;
    
    public StowPosition(
        ControlArmSubsystem controlArm,
        IntakeSubsystem intake) {
        addRequirements(controlArm, intake);
        m_controlArm = controlArm;
        m_intake = intake;
    }

    @Override
    public void execute() {
        m_intake.setRotation(Constants.Position.StowPosition.IntakePosition);
        m_controlArm.stopLowerArm();
        m_controlArm.stopUpperArm();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
