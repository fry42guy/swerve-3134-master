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
  //private final double m_zrotation;
  private  double m_Current_Dist;
  private final ChassisSpeeds speed;
  private TalonFX m_TalonFX1 = new TalonFX(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR);

  private final ChassisSpeeds stop;
  //private final Timer m_time;
  
 
 private double Distance_To_Travel_Target;

 private double Current_Dist_Meters;
 private double Ticks_To_Meters_Multiplyer;
 private double MK4LV2_GearRatio;
 private double ticks_per_Rev;
 private double Wheel_Diameter_Meters;

 //2048 ticks / rev
 //1/6.75 Gear Ratio
 //


  private final DrivetrainSubsystem m_drivetrainSubsystem;
  /** Creates a new AutoDrive. */
  public AutoDrive_For_Distance(DrivetrainSubsystem drivetrainSubsystem ,double xdrive, double ydrive, double Meter_Dist_Target) {


    this.m_drivetrainSubsystem = drivetrainSubsystem;
this.m_xdrive = xdrive;
this.m_ydrive = ydrive;

this.Distance_To_Travel_Target = Meter_Dist_Target;


Wheel_Diameter_Meters = 0.10033;
ticks_per_Rev = 2048;
MK4LV2_GearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
Ticks_To_Meters_Multiplyer = (((Wheel_Diameter_Meters*Math.PI)*MK4LV2_GearRatio)/ticks_per_Rev);


    addRequirements(drivetrainSubsystem);
    speed = new ChassisSpeeds(xdrive,ydrive,0);
    stop = new ChassisSpeeds(0,0,0);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    m_TalonFX1.setSelectedSensorPosition(0);

    
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("this is running");

    m_drivetrainSubsystem.drive(speed);

  Current_Dist_Meters();

  
  //end(true);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("this is end");
    m_drivetrainSubsystem.drive(stop);
    
  }

  private void Current_Dist_Meters(){
Current_Dist_Meters= (Math.abs(m_TalonFX1.getSelectedSensorPosition()*Ticks_To_Meters_Multiplyer));
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
    if(Distance_To_Travel_Target<Current_Dist_Meters ){
      return true;
    }
else
    return false;
  }
}
