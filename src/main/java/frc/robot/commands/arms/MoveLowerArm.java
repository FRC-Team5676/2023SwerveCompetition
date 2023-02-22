package frc.robot.commands.arms;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ControlArmSubsystem;

public class MoveLowerArm extends CommandBase {

    private final ControlArmSubsystem m_controlArm;
    private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
    private final DoubleSupplier m_throttleInput;

    /** Driver control */
    public MoveLowerArm(ControlArmSubsystem controlArm, DoubleSupplier throttleInput) {
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
        double throttle = m_throttleInput.getAsDouble() * Math.signum(m_throttleInput.getAsDouble());

        // square values while keeping original sign
        throttle = -Math.signum(throttle) * Math.pow(throttle, 2);

        double throttle_sl = m_slewX.calculate(throttle);

        m_controlArm.driveLowerArm(throttle_sl);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}