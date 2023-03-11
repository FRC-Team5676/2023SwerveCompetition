package frc.robot.commands.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends CommandBase {

  private Pose2d m_pose;
  private DriveSubsystem m_driveSubsystem;

  // Command constructor
  public ResetOdometry(DriveSubsystem driveSubsystem, Pose2d pose){
    m_driveSubsystem = driveSubsystem;
    m_pose = pose;
  }

  // Reset robot position when command starts
  @Override
  public void initialize() 
  {
    m_driveSubsystem.resetOdometry(m_pose);
  }

  // Stop command once it starts 
  @Override
  public boolean isFinished()
  {
    return true;
  }

}