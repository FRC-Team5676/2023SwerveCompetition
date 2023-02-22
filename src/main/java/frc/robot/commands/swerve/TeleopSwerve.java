package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.controllers.joystick;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean _fieldRelative;
    private DriveSubsystem _swerve;
    private joystick _driver;

    /** Driver control */
    public TeleopSwerve(DriveSubsystem swerve, joystick driver) {
        _swerve = swerve;
        addRequirements(_swerve);

        _driver = driver;
        _fieldRelative = Constants.CustomConstants.fieldRelative;
    }

    @Override
    public void execute() {
        if (RobotContainer.isAutoTargetOn)
            return;

        double yAxis = _driver.getLeftStickY();
        double xAxis = _driver.getLeftStickX();
        double rAxis = _driver.getRightStickX();

        translation = new Translation2d(yAxis, xAxis).times(Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        rotation = rAxis * Constants.DriveConstants.kMaxAngularSpeed;
        _swerve.drive(translation, rotation, _fieldRelative);
    }
}
