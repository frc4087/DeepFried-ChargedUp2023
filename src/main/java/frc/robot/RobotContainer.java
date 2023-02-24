// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final XboxController driveJoy = new XboxController(0);
  public final XboxController opJoy = new XboxController(1);
  public final JoystickButton aButton = new JoystickButton(driveJoy,1);
  public final JoystickButton bButton = new JoystickButton(driveJoy,2);
  public final JoystickButton xButton = new JoystickButton(driveJoy, 3);
  public final JoystickButton startButton = new JoystickButton(opJoy,8);
  //public final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
 // public final ProfiledPIDController m_PidController = new ProfiledPIDController(1.3,0.0,0.7, m_Constraints, 0.02);

  public final ElevatorBase m_elevatorBase = new ElevatorBase();


  public double getDriveJoy(int axis) {
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }
  public double getDriveJoyXR() {
    double raw = getDriveJoy(4);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getDriveJoyYL() {
    double raw = getDriveJoy(1);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }


  public RobotContainer() {
   
  

  }

  public void teleopPeriodic(){

    double setpoint = -40;

    
    SmartDashboard.putNumber("Encoder Right", m_elevatorBase.encoderR.getPosition());
    SmartDashboard.putNumber("PID Method",m_elevatorBase.m_PidController.calculate(m_elevatorBase.encoderR.getPosition()));


    if(driveJoy.getBButtonPressed()){
      m_elevatorBase.m_PidController.setGoal(setpoint);
    }else if(driveJoy.getXButtonPressed()){
      m_elevatorBase.m_PidController.setGoal(0);
    }

    if(driveJoy.getAButton()){
      m_elevatorBase.encoderR.setPosition(0);
    }

    m_elevatorBase.rightSpark.set(m_elevatorBase.m_PidController.calculate(m_elevatorBase.encoderR.getPosition()));
    
    // if(driveJoy.getAButton()){
    //   m_elevatorBase.rightSpark.set(m_elevatorBase.elevatorPID.calculate(m_elevatorBase.encoderR.getPosition(),setpoint)); 
    // }
   

  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
