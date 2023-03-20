// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
/** Add your docs here. */
public class Swerve_Odometry {
    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;
    private final CANCoder m_turningEncoder;
    private static final double kWheelRadius = 0.050165;
    private static final int kEncoderResolution = 4096;
    private static final double kDriveEncoderResolution = 2048*6.75;
    private static final double Unknown_Multiplier_That_works = 4.572/3.3718;


    private static double m_angleoffset;
public Swerve_Odometry(
    int driveMotorChannel,
    int turningMotorChannel,
    int turningEncoderChannel,
    double AngleOffset
){
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_turningEncoder = new CANCoder(turningEncoderChannel);
    m_angleoffset = AngleOffset;
    
}
    


public SwerveModuleState getState() {
    
    return new SwerveModuleState(
        m_driveMotor.getSelectedSensorVelocity()*((2 * Math.PI * kWheelRadius) / kDriveEncoderResolution)*Unknown_Multiplier_That_works, new Rotation2d((m_turningEncoder.getPosition() + m_angleoffset )*(Math.PI/180))); //.087890625*
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getSelectedSensorPosition()*((2 * Math.PI * kWheelRadius) / kDriveEncoderResolution)*Unknown_Multiplier_That_works, new Rotation2d((m_turningEncoder.getPosition() + m_angleoffset)*(Math.PI/180))); //.087890625*
  }

public void ResetDriveEncoder(){
m_driveMotor.setSelectedSensorPosition(0.0);

}






}
