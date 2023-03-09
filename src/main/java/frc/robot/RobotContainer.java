// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeRelease;
import frc.robot.commands.Preload;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.IntakeSub;

import java.io.IOException;
import java.nio.file.Path;

import org.opencv.core.Point;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

  public final POVButton upPOV = new POVButton(opJoy, 90);
  public final POVButton downPOV = new POVButton(opJoy, 0);

  public final ElevatorPID m_elevatorPID = new ElevatorPID();
  public final DriveBase m_DriveBase = new DriveBase();
  public final IntakeSub m_intakeSub = new IntakeSub();


  public Trajectory trajectory;
  public Command m_autonomousCommand;
  public SendableChooser<String> autoChooser = new SendableChooser<String>();


  int lastHeld;
  static final int CONE = 1;
  static final int CUBE = 2;
  public boolean BButtonToggle = false;




  public double getDriveJoy(int axis) {
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < 0.1 ? 0.0 : raw;
  }
  public double getDriveJoyXR() {
    double raw = getDriveJoy(4);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getDriveJoyYL(){
    double raw = getDriveJoy(1);
    return raw;
    //return Math.abs(raw) < 0.1 ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }


  




  public RobotContainer() {
   
  

  }


  public void roboInit(){

    autoChooser.addOption("MobilityPath1","MobilityPath1");
    autoChooser.addOption("MobilityPathR","MobilityPathR");
    autoChooser.addOption("Score Preload", "PreloadPath1B");
    autoChooser.addOption("DockPath", "DockPath");
    SmartDashboard.putData("Auto Routine", autoChooser);


    

    
  }

  public void autoInit(){
    m_DriveBase.m_gyro.reset();
    m_DriveBase.resetEncoders();



    if (autoChooser.getSelected() != null){
      m_autonomousCommand = getAutonomousCommand(autoChooser.getSelected());
      m_autonomousCommand.schedule();
    }
  
  }

  public void autoPeriodic(){
    m_DriveBase.m_drive.feed();
    //m_DriveBase.m_drive.setSafetyEnabled(false);
    
    


  }

  public void teleOperatedInit(){
    m_DriveBase.resetEncoders();

   //m_elevatorPID.enable();
   m_elevatorPID.encoderR.setPosition(0);
   //m_intakePIDSub.intakeEncoder.setPosition(0);
   lastHeld = 3;
   
  }

  public void teleoperatedPeriodic(){


    SmartDashboard.putNumber("Encoder Right", m_elevatorPID.encoderR.getPosition());
    SmartDashboard.putNumber("Talon Left",m_DriveBase.leftEncPos);
    SmartDashboard.putNumber("Talon Right",m_DriveBase.rightEncPos);

  //Driving Junk
    if (driveJoy.getBButtonPressed()){
      BButtonToggle = !BButtonToggle;
    }

    if (BButtonToggle) {
      m_DriveBase.m_drive.curvatureDrive(getDriveJoyYL(), -getDriveJoyXR(), true);
      SmartDashboard.putString("Drivetype", "curvature");
    } else {
      m_DriveBase.m_drive.arcadeDrive(getDriveJoyYL(), -getDriveJoyXR());
      SmartDashboard.putString("Drivetype", "arcade");
    }


  //Local Variables - Elevator Setpoints 
    double setpoint1 = -20;
    double setpoint2 = -54;

  //BUTTON CONTROLS 

   //----Elevator-----------
    if(opJoy.getBButton()){
      m_elevatorPID.setGoal(setpoint1);
      m_elevatorPID.enable();
    }else if(opJoy.getYButton()){
      m_elevatorPID.setGoal(setpoint2);;
    }else if(opJoy.getAButton()){
      m_elevatorPID.setGoal(0);
      m_elevatorPID.enable();
    }

    //----Intake Rollers ---- 
    if(opJoy.getRightBumper()){
      m_intakeSub.intakeRoll.set(1);
      lastHeld = CONE;
    }else if(opJoy.getLeftBumper()){
      m_intakeSub.intakeRoll.set(-1);
      lastHeld = CUBE;
    }else if(lastHeld == CONE){
      m_intakeSub.intakeRoll.set(0.2);
    }else if(lastHeld == CUBE){
       m_intakeSub.intakeRoll.set(-0.2);
    }else{
      m_intakeSub.intakeRoll.set(0);
    }

    if(opJoy.getBackButton()){
      m_intakeSub.intakeRaise.set(0.2);
    }else if(opJoy.getStartButton()){
      m_intakeSub.intakeRaise.set(-0.6);
    }

    }

  

//Disables and resets elevator PID 
public void disableElevatorPID(){
  m_elevatorPID.disable();
}


//Commands


//Automated path following 

public Command getAutonomousCommand(String path) {
  switch(path){

  case "MobilityPath1":
    return pathFollow("output/MobilityPath1.wpilib.json", false);
  case "MobilityPathR":
    return pathFollow("output/MobilityPathR.wpilib.json",false);
  case "PreloadPath1B":
   return new IntakeRelease()
    .alongWith(pathFollow("output/PreloadPath1B.wpilib.json",false));
  case "DockPath":
    return pathFollow("output/DockPath.wpilib.json",false);
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






