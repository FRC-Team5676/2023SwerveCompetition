// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arms.MoveUpperArmCommand;
import frc.robot.commands.arms.MoveLowerArmCommand;
import frc.robot.commands.arms.RotateIntakeCommand;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.swerve.TeleopSwerveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.UpperArmSubsystem;
import frc.robot.utils.AutonManager;
import frc.robot.controllers.xbox;
import frc.robot.controllers.joystick;

public class RobotContainer {
  // Autonomous manager import
  private final AutonManager autonManager = new AutonManager();

  // The robot's subsystems
  private final DriveSubsystem swerve = new DriveSubsystem();
  private final LowerArmSubsystem lowerArm = new LowerArmSubsystem();
  private final UpperArmSubsystem upperArm = new UpperArmSubsystem();
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
    autonManager.addDefaultOption("Set Cone and Leave", AutoRoutines.PlaceConeAndLeave(lowerArm, upperArm, intakeArm, swerve));
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
    operator.buttonA.onTrue(new InstantCommand(swerve::toggleFieldRelative));
    operator.buttonY.onTrue(new InstantCommand(swerve::zeroGyro));
    
    driver.button7.onTrue(new InstantCommand(swerve::toggleFieldRelative));
    driver.button8.onTrue(new InstantCommand(swerve::zeroGyro));

    lowerArm.setDefaultCommand(new MoveLowerArmCommand(lowerArm, operator));
    upperArm.setDefaultCommand(new MoveUpperArmCommand(upperArm, operator) );
    intakeArm.setDefaultCommand(new RotateIntakeCommand(intakeArm, operator));
  }
}
