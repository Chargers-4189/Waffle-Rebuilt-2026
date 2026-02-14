// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double kROTATIONAL_POWER = 0.8;
    public static final double kDRIVE_POWER = 5;
  }

  public static class FlyWheelConstants {
    public static final int flyWheelID = 1;

    //slot 0 configs
    public static final double kS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 0.11; // A position error of 2.5 rotations results in 12 V output
    public static final double kI = 0; // no output for integrated error
    public static final double kD = 0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    public static final double MotionMagicAcceleration = 400; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final double MotionMagicJerk = 4000; // Target jerk of 1600 rps/s/s (0.1 seconds)

  }
}
