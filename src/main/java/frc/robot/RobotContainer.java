// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arms.MoveArmsCommand;
import frc.robot.commands.arms.RotateIntakeCommand;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.swerve.TeleopSwerveCommand;
import frc.robot.subsystems.ControlArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.AutonManager;
import frc.robot.controllers.xbox;
import frc.robot.controllers.joystick;

public class RobotContainer {
  // Autonomous manager import
  private final AutonManager autonManager = new AutonManager();

  // The robot's subsystems
  private final DriveSubsystem swerve = new DriveSubsystem();
  private final ControlArmSubsystem controlArm = new ControlArmSubsystem();
  private final IntakeSubsystem intakeArm = new IntakeSubsystem();

  // The driver's controller
  private final joystick driver = new joystick(1);
  private final xbox operator = new xbox(0);

  public static boolean isAutoTargetOn = false;

  public RobotContainer() {
    addAutonomousChoices();
    autonManager.displayChoices();

    configureButtonBindings();
  }

  public Command getAutonomousCommand() {
    return autonManager.getSelected();
  }

  private void addAutonomousChoices() {
    autonManager.addDefaultOption("Set Cone and Leave", AutoRoutines.PlaceConeAndLeave(controlArm, intakeArm, swerve));
    //autonManager.addOption("Do Nothing", new InstantCommand());
    // autonManager.addOption("PathPlanner Test", new PathPlannerAuto(swerve, controlArm, intake));
  }

  private void configureButtonBindings() {
    swerve.setDefaultCommand(
        new TeleopSwerveCommand(
            swerve,
            () -> driver.getStickY(),
            () -> driver.getStickX(),
            () -> driver.getStickZ()));

    //operator.buttonA.onTrue(new InstantCommand(swerve::toggleSwerveMode));
    operator.buttonY.onTrue(new InstantCommand(swerve::zeroGyro));

    controlArm.setDefaultCommand(new MoveArmsCommand(controlArm, operator));
    intakeArm.setDefaultCommand(new RotateIntakeCommand(intakeArm, operator));
  }
}
