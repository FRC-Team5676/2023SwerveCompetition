// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arms.MoveUpperArmCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.arms.MoveLowerArmCommand;
import frc.robot.commands.arms.RotateIntakeCommand;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.SwervePath1;
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

  // Create PID controllers for trajectory tracking
  public final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  public final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private final PIDController zController = new PIDController(AutoConstants.kPZController, 0, 0);

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
    //autonManager.addOption("Set Cone and Leave",
    //    AutoRoutines.PlaceConeAndLeave(lowerArm, upperArm, intakeArm, swerve));
    autonManager.addDefaultOption("Swerve Path 1", 
        new SwervePath1(swerve, xController, yController, zController));
  }

  private void configureButtonBindings() {
    swerve.setDefaultCommand(
        new TeleopSwerveCommand(
            swerve,
            () -> driver.getStickY(),
            () -> driver.getStickX(),
            () -> driver.getStickZ()));

    // operator.buttonA.onTrue(new InstantCommand(swerve::toggleSwerveMode));
    // operator.leftBumper.and(operator.buttonY).onTrue(new
    // InstantCommand(upperArm::moveToMidPosition));
    operator.buttonY.onTrue(new InstantCommand(upperArm::moveToMaxPosition));
    operator.buttonA.onTrue(new InstantCommand(upperArm::moveToBottomPosition));

    operator.buttonB.onTrue(new InstantCommand(lowerArm::moveToFarPosition));
    operator.buttonX.onTrue(new InstantCommand(lowerArm::moveToBackPosition));

    driver.button7.onTrue(new InstantCommand(swerve::toggleFieldRelative));
    driver.button8.onTrue(new InstantCommand(swerve::zeroGyro));

    lowerArm.setDefaultCommand(new MoveLowerArmCommand(lowerArm, operator));
    upperArm.setDefaultCommand(new MoveUpperArmCommand(upperArm, operator));
    intakeArm.setDefaultCommand(new RotateIntakeCommand(intakeArm, operator));
  }
}
