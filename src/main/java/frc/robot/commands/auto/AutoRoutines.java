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
                new StartEndCommand(() -> arms.moveLowerArmToPosition(-0.6), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(1),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(6), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(2),
                new StartEndCommand(() -> arms.moveLowerArmToPosition(0.3), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(1),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(5), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(0.5),
                new StartEndCommand(() -> robot.drive(0.5, 0, 0, true), () -> robot.drive(0, 0, 0, true), robot)
                        .withTimeout(0.5),
                new StartEndCommand(() -> intake.moveIntakeToPosition(1.35), () -> intake.rotateIntake(0), intake)
                        .withTimeout(1),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(0), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(2),
                new StartEndCommand(() -> arms.moveLowerArmToPosition(0.1), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(1),
                new StartEndCommand(() -> intake.moveIntakeToPosition(0.5), () -> intake.rotateIntake(0), intake)
                        .withTimeout(1),
                new StartEndCommand(() -> robot.drive(0.5, 0, 0, true), () -> robot.drive(0, 0, 0, true), robot)
                        .withTimeout(2));
    }
}
