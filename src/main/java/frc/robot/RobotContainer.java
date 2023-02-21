// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.accessories.OperateControlArm;
import frc.robot.commands.accessories.OperateIntake;
//import frc.robot.commands.auto.PathPlannerAuto;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.ControlArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.AutonManager;
import frc.robot.controllers.xbox;
import frc.robot.controllers.joystick;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Autonomous manager import
  private final AutonManager autonManager = new AutonManager();

  // The robot's subsystems
  private final DriveSubsystem swerve = new DriveSubsystem();
  //private final LimelightSubsystem Limelight = new LimelightSubsystem();
  private final ControlArmSubsystem controlArm = new ControlArmSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  // The driver's controller
  private final joystick driver = new joystick(0);
  private final xbox operator = new xbox(1);

  public static boolean isAutoTargetOn = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutonomousChoices();
    autonManager.displayChoices();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void addAutonomousChoices() {
    autonManager.addOption("Do Nothing", new InstantCommand());
    //autonManager.addOption("PathPlanner Test", new PathPlannerAuto(swerve, controlArm, intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling
   * passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.

    final int translationAxis = XboxController.Axis.kLeftY.value;
    final int strafeAxis = XboxController.Axis.kLeftX.value;
    final int rotationAxis = XboxController.Axis.kRightX.value;

    swerve.setDefaultCommand(
        new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis));

    driver.buttonA.onTrue(new InstantCommand(swerve::toggleSwerveMode));
    driver.buttonY.onTrue(new InstantCommand(swerve::zeroGyro));

    controlArm.setDefaultCommand(new OperateControlArm(operator, controlArm));
    intake.setDefaultCommand(new OperateIntake(operator, intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonManager.getSelected();
  }
}
