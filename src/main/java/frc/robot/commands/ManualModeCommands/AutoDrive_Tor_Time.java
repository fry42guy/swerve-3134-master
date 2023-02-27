// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualModeCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrive_Tor_Time extends CommandBase {

  private final double m_xdrive;
  private final double m_ydrive;
  private final double m_zrotation;
  private final double m_seconds;

  private final ChassisSpeeds speed;

  private final ChassisSpeeds stop;
  //private final Timer m_time;
  
 private final Timer m_time = new Timer();


  private final DrivetrainSubsystem m_drivetrainSubsystem;
  /** Creates a new AutoDrive. */
  public AutoDrive_Tor_Time(DrivetrainSubsystem drivetrainSubsystem ,double xdrive, double ydrive, double zRotation, double timeseconds) {


    this.m_drivetrainSubsystem = drivetrainSubsystem;
this.m_xdrive = xdrive;
this.m_ydrive = ydrive;
this.m_zrotation = zRotation;
this.m_seconds = timeseconds;






    addRequirements(drivetrainSubsystem);
    speed = new ChassisSpeeds(xdrive,ydrive,zRotation);
    stop = new ChassisSpeeds(0,0,0);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    m_time.restart();
    System.out.println("driveinit");
    System.out.println(m_seconds);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("this is running");

    m_drivetrainSubsystem.drive(speed.fromFieldRelativeSpeeds(speed,m_drivetrainSubsystem.getGyroscopeRotation()));
System.out.println(m_time.get());
  
  //end(true);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("this is end");
    m_drivetrainSubsystem.drive(stop);
    m_time.stop();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
    if(m_time.get() >= m_seconds){
      return true;
    }
else
    return false;
  }


  
}
