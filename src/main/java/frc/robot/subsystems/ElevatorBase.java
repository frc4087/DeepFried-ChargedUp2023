// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ElevatorBase extends SubsystemBase {
  public final CANSparkMax rightSpark = new CANSparkMax(15,MotorType.kBrushless);
  public final CANSparkMax leftSpark = new CANSparkMax(14, MotorType.kBrushless);
  public final RelativeEncoder encoderR = rightSpark.getEncoder();
  public final RelativeEncoder encoderL = leftSpark.getEncoder();
  //public final PIDController elevatorPID = new PIDController(0.01, 0, 0);
 public final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(200, 200);
 public final ProfiledPIDController m_PidController = new ProfiledPIDController(0.07,0.0,0.0, m_Constraints, 0.02);
   
 //kP = 0.05
 
  /** Creates a new ElevatorBase. */
  public ElevatorBase() {

    leftSpark.follow(rightSpark, true);

   

   

   

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
