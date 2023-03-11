package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.util.ResetOdometryInverse;
import frc.robot.commands.util.ResetYaw;
import frc.robot.subsystems.DriveSubsystem;

public class SwervePath1 extends SequentialCommandGroup {

    private final PathPlannerTrajectory path = PathPlanner.loadPath("PowerUpAuto-1", new PathConstraints(0.5, 0.5));

    public SwervePath1(DriveSubsystem driveSubsystem, PIDController xController,
            PIDController yController, PIDController ppthetaController) {

        addCommands(
                //new ElevatorSolenoid(elevatorSubsystem),
                //new WaitCommand(1.5),
                //new ElevatorZero(elevatorSubsystem, grabberSubsystem),
                //new WaitCommand(1),
                new TrajectoryWeaver(driveSubsystem, xController, yController, ppthetaController, path, true),
                //new GrabberSolenoid(grabberSubsystem),
                new WaitCommand(2),
                new ResetYaw(driveSubsystem),
                new ResetOdometryInverse(driveSubsystem));
    }
}