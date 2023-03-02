// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class IntakePID extends ProfiledPIDSubsystem {
  
  public final CANSparkMax intakeRoll = new CANSparkMax(21, MotorType.kBrushless);
  public final CANSparkMax intakeRaise = new CANSparkMax(22,MotorType.kBrushless);
  public final RelativeEncoder intakeEncoder = intakeRaise.getEncoder();
  public final RelativeEncoder rollEncoder = intakeRoll.getEncoder();
 


  /** Creates a new IntakePID. */
  public IntakePID() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.05,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(10, 10)));

    intakeRoll.setSmartCurrentLimit(20);
    intakeRaise.setSmartCurrentLimit(20);
  //  intakeRoll.setIdleMode(IdleMode.kBrake);
  //  double sparkEncPos = intakeRoll.setSoftLimit(null, 0);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    intakeRaise.set(output);
  }

  public double getEncoderPosIntake(){
    return intakeEncoder.getPosition();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderPosIntake();
  }
}
