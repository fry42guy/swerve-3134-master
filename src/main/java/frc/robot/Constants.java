// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .58; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .595; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0); // FIXME Measure and set back right steer offset



//###############################################################################################################################
public static final double Swereve_Front_Angle_Offset = 90; //90   

public static final int PCM_CAN_ID = 1 ;
    public static final int Arm_Intake_Left_ID = 51;
    public static final int Arm_Intake_Right_ID = 50;
    public static final double Arm_intake_speed = .30;


    public static final int m_Wrist = 44; // ArmMotor Falcon 500 CAN ID ###
    public static final double Wrist_Motor_Speed = .2; 
    public static final double Wrist_Limit_High = 123000;
    public static final double Wrist_Limit_Low = 0;

    public static final int m_Vertical = 45; // ArmMotor Falcon 500 CAN ID ###
    public static final double Vertical_Motor_Speed = .4; 
    public static final double Vertical_Motio_Accel = 100; //units per 100ms - 10- 100ms/sec 4000units/rev 
    public static final double Vertical_Limit_High = 0;
    public static final double Vertical_limit_Low = -190000;
    
    public static final double Vertical_High_Setpoint = -180000;
    public static final double Vertical_Low_Setpoint = -4000;

    public static final double Vertical_PID_Tolerance_Offset = 0;
    public static final double Horizontal_PID_Tolerance_Offset = 0;
    public static final double Wrist_PID_Tolerance_Offset = 0;


    public static final double Wrist_High_Setpoint = 105000;
    public static final double Wrist_cube_Highth = 100000;
    public static final double Wrist_Low_Setpoint = 10000;
    public static final double Wrist_PID_Speed = .3;


    public static final int m_Horizontal = 46; // ArmMotor Falcon 500 CAN ID ###
    public static final double Horizontal_Motor_Speed = .2; 
    public static final double Horizontal_PID_Speed = .4;
    public static final double Horizontal_Limit_High = 105000 ;
    public static final double Horizontal_Limit_Low = 0;

// Start/Stow

public static final double Store_Stoe_Vert = -4000;
public static final double Store_Stoe_Wrist = 9252;
public static final double Store_Stoe_Hori = 100;

//Floor Pick up Cube/Cone

public static final double Floor_Cube_Cone_Vert = -4000;
public static final double Floor_Cube_Cone_Wrist = 113771;
public static final double Floor_Cube_Cone_Hori = 100;
//Score Cone/Cube MID

public static final double Cone_Cube_MID_Vert = -177192;
public static final double Cone_Cube_MID_Wrist = 91095;
public static final double Cone_Cube_MID_Hori = 100082;


//Score Cone/Cube High
public static final double Cone_Cube_High_Vert = -168192;
public static final double Cone_Cube_High_Wrist = 62000;
public static final double Cone_Cube_High_Hori = 100018;

//Travel w/Cone Cube

public static final double Cone_Cube_Travel_Vert = -4000;
public static final double Cone_Cube_Travel_Wrist = 17000;
public static final double Cone_Cube_Travel_Hori = 100;

//Pickup Player Station 

public static final double Cone_Cube_Player_Station_Vert = -177192;
public static final double Cone_Cube_Player_Station_Wrist = 79000;
public static final double Cone_Cube_Player_Station_Hori = 100;


public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2),
    new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2),
    new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, DRIVETRAIN_TRACKWIDTH_METERS / 2),
    new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2));


    public static final class AutoConstants {
    public static final HashMap<String, Command> eventMap = new HashMap<>();

    }





}
