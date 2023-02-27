// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualModeCommands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrive_For_Distance extends CommandBase {

  private final double m_xdrive;
  private final double m_ydrive;
  private final double m_zrotation;
  private  double m_Current_Dist;
  private final ChassisSpeeds speed;
  private TalonFX m_TalonFX1 = new TalonFX(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR);

  private final ChassisSpeeds stop;
  //private final Timer m_time;
  
 private final Timer m_time = new Timer();
 private final double Distance_ToTravel;
 private double init_Distance;
 private double end_Dist;


  private final DrivetrainSubsystem m_drivetrainSubsystem;
  /** Creates a new AutoDrive. */
  public AutoDrive_For_Distance(DrivetrainSubsystem drivetrainSubsystem ,double xdrive, double ydrive, double zRotation, double Meter_Dist) {


    this.m_drivetrainSubsystem = drivetrainSubsystem;
this.m_xdrive = xdrive;
this.m_ydrive = ydrive;
this.m_zrotation = zRotation;
this.Distance_ToTravel = Meter_Dist;






    addRequirements(drivetrainSubsystem);
    speed = new ChassisSpeeds(xdrive,ydrive,zRotation);
    stop = new ChassisSpeeds(0,0,0);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    m_time.restart();
    init_Distance = m_TalonFX1.getSelectedSensorPosition();
    end_Dist = Math.abs(init_Distance)+Math.abs(Distance_ToTravel);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("this is running");

    m_drivetrainSubsystem.drive(speed);
System.out.println(m_time.get());

m_Current_Dist = m_TalonFX1.getSelectedSensorPosition();


  
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

    
    if(end_Dist<Distance_ToTravel){
      return true;
    }
else
    return false;
  }
}
