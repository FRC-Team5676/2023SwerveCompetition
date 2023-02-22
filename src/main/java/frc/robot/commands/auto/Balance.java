/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends CommandBase {

    private final DriveSubsystem _swerve;

    private double _rotation;

    private double _error;
    private double _currentAngle;
    private double _drivePower;

    /**
     * Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance.
     *
     * @param swerve
     */
    public Balance(DriveSubsystem swerve) {
        addRequirements(swerve);
        _swerve = swerve;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    @Override
    public void execute() {

        this._currentAngle = _swerve.getPitch();

        _error = Constants.Balance.BEAM_BALANCED_GOAL_DEGREES - _currentAngle;
        _drivePower = -Math.min(Constants.Balance.BEAM_BALANACED_DRIVE_KP * _error, 1);

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (_drivePower < 0) {
            _drivePower *= Constants.Balance.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
        }

        // Limit the max power
        if (Math.abs(_drivePower) > 0.4) {
            _drivePower = Math.copySign(0.4, _drivePower);
        }

        _swerve.drive(_drivePower, 0, _rotation, true);

        // Debugging Print Statments
        System.out.println("Current Angle: " + _currentAngle);
        System.out.println("Error " + _error);
        System.out.println("Drive Power: " + _drivePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(_error)
                < Constants.Balance
                        .BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the
        // specified threshold of being 'flat' (gyroscope
        // pitch of 0 degrees)
    }
}
