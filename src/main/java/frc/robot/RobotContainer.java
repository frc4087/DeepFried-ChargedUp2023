// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.IntakePID;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
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
  public final DigitalInput hallEffect = new DigitalInput(0);
  //public final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
 // public final ProfiledPIDController m_PidController = new ProfiledPIDController(1.3,0.0,0.7, m_Constraints, 0.02);

  public final ElevatorPID m_elevatorPID = new ElevatorPID();
  public final IntakePID m_intakePIDSub = new IntakePID();

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

  public void roboInit(){
    

    
  }

  public void teleOpInit(){
   // m_elevatorPID.enable();
   m_elevatorPID.encoderR.setPosition(0);
  }

  public void teleopPeriodic(){

    double setpoint = -20;
    double raise = Math.PI/4;
    double lower = Math.PI/6;

    SmartDashboard.putNumber("Encoder Right", m_elevatorPID.encoderR.getPosition());
    SmartDashboard.putNumber("IntakeRaise Encoder",m_intakePIDSub.intakeEncoder.getPosition());
   // SmartDashboard.putBoolean("Hall Effect Validity", getHallEffect());
    //SmartDashboard.putNumber("PID Method",m_elevatorBase.m_PidController.calculate(m_elevatorPID.encoderR.getPosition()));

  
    if(driveJoy.getBButton()){
      m_elevatorPID.setGoal(setpoint);
      m_elevatorPID.enable();
    }else if(driveJoy.getXButton()){
      m_elevatorPID.setGoal(0);
      m_elevatorPID.enable();
    }

    if(driveJoy.getYButton()){
      m_intakePIDSub.setGoal(-raise);
      m_intakePIDSub.enable();
    }else if(driveJoy.getAButton()){
      m_intakePIDSub.setGoal(lower);
      m_intakePIDSub.enable();
    }
    // if(driveJoy.getAButton()){
    //   m_elevatorPID.encoderR.setPosition(0);
    // }

  

   //m_elevatorPID.rightSpark.set(m_elevatorPID.useOutput(setpoint,setpoint));

    
    
   

    
}

public void disableElevatorPID(){
  m_elevatorPID.disable();
  m_intakePIDSub.disable();
}

// public boolean getHallEffect(){
//   return !hallEffect.get();
// }

// public void resetEncoder(){
//   if (getHallEffect()){
//     m_elevatorPID.encoderR.setPosition(0);
//   }


}


