package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopSwerve extends CommandBase {

    private final DriveSubsystem m_swerveDrive;
    private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
    private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kTranslationSlew);
    private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kRotationSlew);
    private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

    /** Driver control */
    public TeleopSwerve(
            DriveSubsystem swerveDriveSubsystem,
            DoubleSupplier throttleInput,
            DoubleSupplier strafeInput,
            DoubleSupplier rotationInput) {
        m_swerveDrive = swerveDriveSubsystem;
        m_throttleInput = throttleInput;
        m_strafeInput = strafeInput;
        m_rotationInput = rotationInput;

        addRequirements(swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = m_throttleInput.getAsDouble() * Math.signum(m_throttleInput.getAsDouble());
        double strafe = m_strafeInput.getAsDouble() * Math.signum(m_strafeInput.getAsDouble());
        double rotation = m_rotationInput.getAsDouble() * Math.signum(m_rotationInput.getAsDouble());

        // square values while keeping original sign
        throttle = -Math.signum(throttle) * Math.pow(throttle, 2);
        strafe = -Math.signum(strafe) * Math.pow(strafe, 2);
        rotation = -Math.signum(rotation) * Math.pow(rotation, 2);

        double throttle_sl = m_slewX.calculate(throttle);
        double strafe_sl = m_slewY.calculate(strafe);
        double rotation_sl = m_slewRot.calculate(rotation);

        m_swerveDrive.drive(throttle_sl, strafe_sl, rotation_sl, true);
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
