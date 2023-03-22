package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDrivePath extends SequentialCommandGroup {

    private final PathPlannerTrajectory path = PathPlanner.loadPath("AutonomousPath3", new PathConstraints(0.5, 0.5));

    public AutonomousDrivePath(DriveSubsystem driveSubsystem, PIDController xController,
            PIDController yController, PIDController ppthetaController) {

        addCommands(
                // new ElevatorSolenoid(elevatorSubsystem),
                // new WaitCommand(1.5),
                // new ElevatorZero(elevatorSubsystem, grabberSubsystem),
                // new WaitCommand(1),
                new TrajectoryGenerator(driveSubsystem, xController, yController,
                        ppthetaController, path, true));
    }
}