package frc.robot.commands.arms;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ControlArmSubsystem;

public class MoveUpperArm extends CommandBase {

    private final ControlArmSubsystem m_controlArm;
    private final SlewRateLimiter m_slew = new SlewRateLimiter(DriveConstants.kTranslationSlew);
    private final DoubleSupplier m_throttleInput;

    /** Driver control */
    public MoveUpperArm(ControlArmSubsystem controlArm, DoubleSupplier throttleInput) {
        m_controlArm = controlArm;
        m_throttleInput = throttleInput;

        addRequirements(controlArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = m_throttleInput.getAsDouble();
        throttle = Math.signum(throttle) * Math.pow(throttle, 2);
        double throttle_sl = m_slew.calculate(throttle);
        m_controlArm.driveUpperArm(throttle_sl);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_controlArm.stopUpperArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
