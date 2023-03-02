// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.IntakePID;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  public final JoystickButton aButton = new JoystickButton(opJoy,1);
  public final JoystickButton bButton = new JoystickButton(opJoy,2);
  public final JoystickButton xButton = new JoystickButton(opJoy, 3);
  public final JoystickButton startButton = new JoystickButton(opJoy,8);
  public final DigitalInput hallEffect = new DigitalInput(0);
  //public final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(1.75, 0.75);
 // public final ProfiledPIDController m_PidController = new ProfiledPIDController(1.3,0.0,0.7, m_Constraints, 0.02);

  public final ElevatorPID m_elevatorPID = new ElevatorPID();
  public final IntakePID m_intakePIDSub = new IntakePID();
  public final DriveBase m_DriveBase = new DriveBase();


  public Trajectory trajectory;


  int lastHeld;
  static final int CONE = 1;
  static final int CUBE = 2;



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

  // public double getOpJoy(int axis){
  //   double raw = opJoy.getRawAxis(axis);
  //   return Math.abs(raw)< 0.1 ? 0.0 : raw;
  // }

  // public double getOpJoyXR() {
  //   double raw = getOpJoy(4);
  //   return raw;
  // }

  // public double getOpJoyYL() {
  //   double raw = getOpJoy(1);
  //   return raw;
  // }




  public RobotContainer() {
   
  

  }

  public void roboInit(){
    

    
  }

  public void teleOpInit(){
   // m_elevatorPID.enable();
   m_elevatorPID.encoderR.setPosition(0);
   m_intakePIDSub.intakeEncoder.setPosition(0);
   lastHeld = 3;
   
  }

  public void teleopPeriodic(){

    double setpoint1 = -20;
    double setpoint2 = -50;


    SmartDashboard.putNumber("Encoder Right", m_elevatorPID.encoderR.getPosition());
    SmartDashboard.putNumber("IntakeRaise Encoder",m_intakePIDSub.intakeEncoder.getPosition());
   // SmartDashboard.putBoolean("Hall Effect Validity", getHallEffect());
    //SmartDashboard.putNumber("PID Method",m_elevatorBase.m_PidController.calculate(m_elevatorPID.encoderR.getPosition()));

  //BUTTON CONTROLS 
    if(opJoy.getBButton()){
      m_elevatorPID.setGoal(setpoint1);
      m_elevatorPID.enable();
    }else if(opJoy.getAButton()){
      m_elevatorPID.setGoal(setpoint2);;
    }else if(opJoy.getXButton()){
      m_elevatorPID.setGoal(0);
      m_elevatorPID.enable();
    }

    // if(opJoy.getYButton()){
    //   m_intakePIDSub.setGoal(-20);
    //   m_intakePIDSub.enable();
    // }else if(opJoy.getAButton()){
    //   m_intakePIDSub.setGoal(-5);
    //   m_intakePIDSub.enable();
    // }


    //these numbers need to be changed for each seperate game piece 
    
    if(opJoy.getRightBumper()){
      m_intakePIDSub.intakeRoll.set(0.9);
      lastHeld = CONE;
    }else if(opJoy.getBButton()){
      m_intakePIDSub.intakeRoll.set(0.9);
      lastHeld = CUBE;
    }else if(lastHeld == CONE){
      m_intakePIDSub.intakeRoll.set(0.2);
    }else if(opJoy.getLeftBumper()){
      m_intakePIDSub.intakeRoll.set(-0.9);
  
    }else if(lastHeld == CUBE){
       m_intakePIDSub.intakeRoll.set(0.2);
    }else{
      m_intakePIDSub.intakeRoll.set(0);
    }

    if(opJoy.getLeftBumper()){
      m_intakePIDSub.intakeRoll.set(1.0);
    } else if(opJoy.getRightBumper()){
      m_intakePIDSub.intakeRoll.set(-1.0);
    } else {
      m_intakePIDSub.intakeRoll.set(0);
    }

    m_DriveBase.m_drive.arcadeDrive(getDriveJoyYL(), -getDriveJoyXR());
    m_intakePIDSub.intakeRaise.set(opJoy.getRawAxis(1));
  

  

    
}





public void disableElevatorPID(){
  m_elevatorPID.disable();
  m_intakePIDSub.disable();
}





public Command getAutonomousCommand(String path) {
  switch(path){

  case "Mobility1":
    return pathFollow("output/Mobility1.wpilib.json", false);
  case "Mobility2":
    return pathFollow("output/Mobility2.wpilib.json", false);
  case "Mobility2Dock":
    return pathFollow("output/Mobility2.wpilib.json", false)
            .andThen(pathFollow("output/Mobility2Rev.wpilib.json", true));
  case "Mobility3":
    return pathFollow("output/Mobility3.wpilib.json", false);
    
  // case "Taxi":
  // //  tracking = false;
  //   return pathFollow("output/Taxi.wpilib.json", false);

  // case "Taxi 1 Ball Low":
  //   //tracking = false;
  //   return //timedLaunchCommand(true, 3)
  //         //.andThen(
  //           pathFollow("output/Taxi.wpilib.json", false);

  // case "Taxi 2 Ball Low":

  //   return //new IntakeActivate()
  //         //.alongWith(
  //           //new ParallelRaceGroup(
  //           //  new BottomFeederActivate(true), 
  //            // new BeamBreakTriggered(1)))
  //        // .alongWith(
  //           pathFollow("output/Taxi.wpilib.json", false)
  //           .andThen(
  //             pathFollow("output/TaxiRev.wpilib.json", true));
  //          // .andThen(
 
  }

  return null;
}

/** 
 * @TODO Auto-generated catch block
*/
public Command pathFollow(String trajectoryJSON, boolean multiPath){
  try {
    Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
  } catch (final IOException ex) {


    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }
  //m_drivebase.m_gyro.reset();
  
  RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
                                                  m_DriveBase::getPose,
                                                  new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                                  new SimpleMotorFeedforward(Constants.ksVolts, 
                                                                             Constants.kvVoltSecondsPerMeter,
                                                                             Constants.kaVoltSecondsSquaredPerMeter),
                                                  Constants.m_driveKinematics,
                                                  m_DriveBase::getWheelSpeeds,
                                                  new PIDController(Constants.kP, 0, 0),
                                                  new PIDController(Constants.kP, 0, 0),
                                                  m_DriveBase::voltageControl,
                                                  m_DriveBase);
  
  // Run path following command, then stop at the end.
  // Robot.m_robotContainer.m_driveAuto.m_drive.feed();
  //m_drivebase.resetOdometry(trajectory.getInitialPose());
  
  if (!multiPath){
    m_DriveBase.resetOdometry(trajectory.getInitialPose());
  } 
  return ramseteCommand;
}
}





