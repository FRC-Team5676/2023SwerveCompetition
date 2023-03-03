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
                new StartEndCommand(() -> arms.moveUpperArmToPosition(40.0), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(5),
                new StartEndCommand(() -> arms.moveLowerArmToPosition(-1.0), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(5),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(163), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(5),
                new StartEndCommand(() -> arms.moveLowerArmToPosition(3.0), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(5),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(145), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(5),
                new StartEndCommand(() -> robot.drive(-0.4, 0, 0, true), () -> robot.drive(0, 0, 0, true), robot)
                        .withTimeout(0.5),
                new StartEndCommand(() -> intake.moveIntakeToPosition(9.23), () -> intake.rotateIntake(0), intake)
                        .withTimeout(5),
                new StartEndCommand(() -> arms.moveUpperArmToPosition(0), () -> arms.driveUpperArm(0), arms)
                        .withTimeout(5),
                new StartEndCommand(() -> arms.moveLowerArmToPosition(0.8), () -> arms.driveLowerArm(0), arms)
                        .withTimeout(5),
                new StartEndCommand(() -> intake.moveIntakeToPosition(4.14), () -> intake.rotateIntake(0), intake)
                        .withTimeout(5),
                new StartEndCommand(() -> robot.drive(-0.5, 0, 0, true), () -> robot.drive(0, 0, 0, true), robot)
                        .withTimeout(1));
    }
}
