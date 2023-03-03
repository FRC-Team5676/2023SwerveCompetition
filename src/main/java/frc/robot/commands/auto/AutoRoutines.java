package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ControlArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRoutines {
    public static Command PlaceConeAndLeave(ControlArmSubsystem arms, IntakeSubsystem intake, DriveSubsystem robot) {
        return Commands.sequence(
                new StartEndCommand(() -> arms.moveUpperArmToPosition(40), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(2),
                new StartEndCommand(() -> arms.moveLowerArmToPosition(-20), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(2),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(163), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(3),
                new StartEndCommand(() -> arms.moveLowerArmToPosition(87), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(2),
                Commands.parallel(
                new StartEndCommand(() -> arms.moveUpperArmToPosition(145), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(2),
                new StartEndCommand(() -> intake.moveIntakeToPosition(10), () -> intake.rotateIntake(0), intake)
                        .withTimeout(2)),
                new StartEndCommand(() -> robot.drive(0.3, 0, 0, true), () -> robot.drive(0, 0, 0, true), robot)
                        .withTimeout(0.75),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(0), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(3),
                Commands.parallel(
                new StartEndCommand(() -> arms.moveLowerArmToPosition(22), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(3),
                new StartEndCommand(() -> robot.drive(0.3, 0, 0, true), () -> robot.drive(0, 0, 0, true), robot)
                        .withTimeout(1)));
    }
}
