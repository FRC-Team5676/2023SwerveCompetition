package frc.robot.commands.arms;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateIntake extends CommandBase {

    private final IntakeSubsystem m_controlArm;
    private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
    private final DoubleSupplier m_throttleInput;

    /** Driver control */
    public RotateIntake(IntakeSubsystem controlArm, DoubleSupplier throttleInput) {
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
        throttle = -Math.signum(throttle) * Math.pow(throttle, 2);
        double throttle_sl = m_slewX.calculate(throttle);
        m_controlArm.rotateIntake(throttle_sl);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_controlArm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
