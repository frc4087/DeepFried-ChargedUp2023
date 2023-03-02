// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kEncoderDistancePerPulse = (4 * Math.PI * 2.54 * 9) / (100.0 * 2048 * 64); //gear ratio is 64 to 9
  public static final double CurrentLimit = 30;
  public static final double secondsForOpenRamp = 0.8;
  public final static int kDriveTimeoutMs = 30,
  kDrivePIDIdx = 0;

  public static final double ksVolts = 0.205,//0.65634, 
  kvVoltSecondsPerMeter = 2.1,//0.1106, 
  kaVoltSecondsSquaredPerMeter = 0.217,//0.095387,
  kTrackwidthMeters = 0.635,
  kP = 1,//0.17833, 
  kD = 0.0, 
  kMaxSpeedMetersPerSecond = 4.953,
  kMaxAccelerationMetersPerSecondSquared = 8.5344,
  kRamseteB = 2, 
  kRamseteZeta = 0.7;



public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);


public static final boolean kGyroReversed = false;



  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
}
