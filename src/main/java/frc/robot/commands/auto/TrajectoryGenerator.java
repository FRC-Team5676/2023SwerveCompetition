package frc.robot.commands.auto;

import java.util.function.Consumer;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.subsystems.DriveSubsystem;

// Runs a given pp-trajectory as a command 
public class TrajectoryGenerator extends SequentialCommandGroup {

    Consumer<ChassisSpeeds> chassisSpeeds;

    // Constructor that obtains required values
    public TrajectoryGenerator(DriveSubsystem driveSubsystem, PIDController xController,
            PIDController yController, PIDController zController,
            PathPlannerTrajectory path, Boolean isFirstPath) {

        // Tell theta PID controller that its a circle
        zController.enableContinuousInput(-180, 180);

        // Check if first path
        if (isFirstPath) {
            // Reset robot odometry before movement
            addCommands(new ResetOdometry(driveSubsystem, path.getInitialHolonomicPose()));
        }

        addCommands(
                new SequentialCommandGroup(
                        // Use Path Planner to move the swerve modules by letting it call
                        // setModuleStatesInAutonomous (Closed Loop)
                        new PPSwerveControllerCommand(path, driveSubsystem::getPose,
                                DriveConstants.kDriveKinematics, xController, yController, 
                                zController, driveSubsystem::setModuleStates, driveSubsystem),

                        // Stop all module movement
                        new InstantCommand(() -> driveSubsystem.stopModules())));
    }
}