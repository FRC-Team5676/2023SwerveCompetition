package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometryInverse extends CommandBase {

  private DriveSubsystem m_driveSubsystem;

  // Command constructor
  public ResetOdometryInverse(DriveSubsystem driveSubsystem){
    m_driveSubsystem = driveSubsystem;
  }

  // Reset robot position when command starts
  @Override
  public void initialize() 
  {
    m_driveSubsystem.resetOdometry(new Pose2d(0.0,0.0, Rotation2d.fromDegrees(180)));
  }

  // Stop command once it starts 
  @Override
  public boolean isFinished()
  {
    return true;
  }

}