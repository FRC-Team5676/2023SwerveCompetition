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

public class AutonomousPath1 extends SequentialCommandGroup {

    private final PathPlannerTrajectory path = PathPlanner.loadPath("AutonomousPath2", new PathConstraints(0.5, 0.5));

    public AutonomousPath1(DriveSubsystem driveSubsystem, PIDController xController,
            PIDController yController, PIDController ppthetaController) {

        addCommands(
                //new ElevatorSolenoid(elevatorSubsystem),
                //new WaitCommand(1.5),
                //new ElevatorZero(elevatorSubsystem, grabberSubsystem),
                //new WaitCommand(1),
                new TrajectoryGenerator(driveSubsystem, xController, yController, ppthetaController, path, true),
                //new GrabberSolenoid(grabberSubsystem),
                new WaitCommand(10),
                new ResetYaw(driveSubsystem),
                new ResetOdometryInverse(driveSubsystem));
    }
}