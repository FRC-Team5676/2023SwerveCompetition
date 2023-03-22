package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.UpperArmSubsystem;

public class AutoRoutines {
        public static Command PlaceConeAndLeave(LowerArmSubsystem lowerArm, UpperArmSubsystem upperArm,
                        IntakeSubsystem intake, DriveSubsystem swerve, PIDController xController,
                        PIDController yController, PIDController zController) {
                return Commands.sequence(
                                new StartEndCommand(() -> lowerArm.moveToPosition(-20),
                                                () -> lowerArm.driveArm(0), lowerArm)
                                                .withTimeout(2),
                                new StartEndCommand(() -> upperArm.moveToPosition(37.3), () -> upperArm.driveArm(0),
                                                upperArm)
                                                .withTimeout(2),
                                new StartEndCommand(() -> lowerArm.moveToPosition(87), () -> lowerArm.driveArm(0),
                                                lowerArm)
                                                .withTimeout(3),
                                Commands.parallel(
                                                new StartEndCommand(() -> upperArm.moveToPosition(33),
                                                                () -> upperArm.driveArm(0), upperArm)
                                                                .withTimeout(2),
                                                new StartEndCommand(() -> intake.moveIntakeToPosition(10),
                                                                () -> intake.rotateIntake(0), intake)
                                                                .withTimeout(2)),
                                new AutonomousDrivePath(swerve, xController, yController, zController),
                                Commands.parallel(
                                                new StartEndCommand(() -> upperArm.moveToPosition(0),
                                                                () -> upperArm.driveArm(0),
                                                                upperArm)
                                                                .withTimeout(3),
                                                new StartEndCommand(() -> lowerArm.moveToPosition(22),
                                                                () -> lowerArm.driveArm(0), lowerArm)
                                                                .withTimeout(3)));
        }
}
