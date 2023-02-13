// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeFWD;
import frc.robot.commands.IntakeREV;
import frc.robot.commands.PIDHorizontalCommand;
import frc.robot.commands.PIDVerticalCommand;
import frc.robot.commands.PIDWristCommand;
import frc.robot.subsystems.AirMod;
import frc.robot.subsystems.ArmIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HorizontalSubsystem;
import frc.robot.subsystems.VerticalSubsystem;
import frc.robot.subsystems.WristSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController m_controller = new XboxController(0);

  private final AirMod M_PCM = new AirMod();

  private final WristSubsystem m_Wrist = new WristSubsystem();

  private final VerticalSubsystem m_Vertical = new VerticalSubsystem();

  private final HorizontalSubsystem m_Horizontal = new HorizontalSubsystem();

  private final ArmIntakeSubsystem m_ArmIntakeSubsystem = new ArmIntakeSubsystem();
 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    M_PCM.Start();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation


    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));




    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope


     new JoystickButton(m_controller, XboxController.Button.kBack.value)
    //         // No requirements because we don't need to interrupt anything
            .onTrue(new InstantCommand (
              ()-> m_drivetrainSubsystem.zeroGyroscope()));  //############################################
            

new JoystickButton(m_controller, XboxController.Button.kBack.value)
.onTrue(new InstantCommand(
  ()-> M_PCM.ClawClose()
));

new JoystickButton(m_controller, XboxController.Button.kStart.value)
.onTrue(new InstantCommand(
  ()-> M_PCM.ClawOpen()
));


   new JoystickButton(m_controller, XboxController.Button.kY.value)
   .onTrue(new InstantCommand(
    () -> m_Wrist.setSpeed(Constants.Wrist_Motor_Speed),
    m_Wrist))
   
    .onFalse(new InstantCommand(
    () -> m_Wrist.stop(),
    m_Wrist));

    new JoystickButton(m_controller, XboxController.Button.kX.value)
    .onTrue(new InstantCommand(
     () -> m_Wrist.setSpeed(Constants.Wrist_Motor_Speed*-1),
     m_Wrist))
    
     .onFalse(new InstantCommand(
     () -> m_Wrist.stop(),
     m_Wrist));
          

   new JoystickButton(m_controller, XboxController.Button.kA.value)
    .onTrue(new InstantCommand(
     () -> m_Vertical.setSpeed(Constants.Vertical_Motor_Speed),
     m_Vertical))
    
     .onFalse(new InstantCommand(
     () -> m_Vertical.stop(),
     m_Vertical));

     new JoystickButton(m_controller, XboxController.Button.kB.value)
    .onTrue(new InstantCommand(
     () -> m_Vertical.setSpeed(Constants.Vertical_Motor_Speed*-1),
     m_Vertical))
    
     .onFalse(new InstantCommand(
     () -> m_Vertical.stop(),
     m_Vertical));

     new Trigger(()->
     
      {
        if(m_controller.getLeftBumper())
          return true;
        else
          return false;
  
      })
      .onTrue(new PIDVerticalCommand(m_Vertical, Constants.Vertical_High_Setpoint));
     
      new Trigger(()->
     
      {
        if(m_controller.getRightBumper())
          return true;
        else
          return false;
  
      })
      .onTrue(new PIDVerticalCommand(m_Vertical, Constants.Vertical_Low_Setpoint));


      new Trigger(()->
     
      {
        if(m_controller.getPOV() == 90)
          return true;
        else
          return false;
  
      })
      .onTrue(new PIDWristCommand(m_Wrist, Constants.Wrist_High_Setpoint));
     
      new Trigger(()->
     
      {
        if(m_controller.getPOV()==270)
          return true;
        else
          return false;
  
      })
      .onTrue(new PIDWristCommand(m_Wrist, Constants.Wrist_Low_Setpoint));



      new Trigger(()->
     
      {
        if(m_controller.getPOV() == 0)
          return true;
        else
          return false;
  
      })
    
      
      .onTrue(new InstantCommand(
        () -> m_Horizontal.setSpeed(Constants.Horizontal_Motor_Speed*-1),
        m_Horizontal))
       
        .onFalse(new InstantCommand(
        () -> m_Horizontal.stop(),
        m_Horizontal));





      new Trigger(()->
     
      {
        if(m_controller.getPOV()==180)
          return true;
        else
          return false;
  
      })
      .onTrue(new InstantCommand(
        () -> m_Horizontal.setSpeed(Constants.Horizontal_Motor_Speed),
        m_Horizontal))
       
        .onFalse(new InstantCommand(
        () -> m_Horizontal.stop(),
        m_Horizontal));




     new Trigger(() ->
     {if(m_controller.getLeftTriggerAxis() >0)
      return true;
      else
      {
        return false;
      }
     
     }
     ).whileTrue(new IntakeFWD(m_ArmIntakeSubsystem));



     new Trigger(() ->
     {if(m_controller.getRightTriggerAxis() >0)
      return true;
      else
      {
        return false;
      }
     
     }
     ).whileTrue(new IntakeREV(m_ArmIntakeSubsystem));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
