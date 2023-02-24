// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeBase extends SubsystemBase {

  //public final TrapezoidProfile.Constraints m_IntakeConstraints = new TrapezoidProfile.Constraints(100, 100);
  public static final SparkMaxAlternateEncoder.Type m_intakeEncoder = SparkMaxAlternateEncoder.Type.kQuadrature;
  public final CANSparkMax m_intakeSpark = new CANSparkMax(10, MotorType.kBrushless);
  


    
  /** Creates a new IntakeBase. */
  public IntakeBase() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
