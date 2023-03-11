package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetYaw extends CommandBase {

  private DriveSubsystem m_driveSubsystem;

  public ResetYaw(DriveSubsystem driveSubsystem){
    m_driveSubsystem = driveSubsystem;
  }

  // Reset robot position when command starts
  @Override
  public void initialize() 
  {
    m_driveSubsystem.zeroGyro();
  }

  // Stop command once it starts 
  @Override
  public boolean isFinished()
  {
    return true;
  }

}