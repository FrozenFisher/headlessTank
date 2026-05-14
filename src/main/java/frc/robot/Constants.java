// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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

  public static class TankConstants {
    public static final int TankLeftMotorID = 2;
    public static final int TankRightMotorID = 1;
    public static final int TankLeftFollowerID = 3;
    public static final MotorAlignmentValue TankLeftFollowerAlignment = MotorAlignmentValue.Aligned;
    public static final int TankRightFollowerID = 4;
    public static final MotorAlignmentValue TankRightFollowerAlignment =
        MotorAlignmentValue.Aligned;

    public static final double kP = 0.1; // TUNE!!
    public static final double kI = 0.;
    public static final double kD = 0.;
    public static final double kV = 1.3; // TUNE!!
    public static final double kS = 0.254;
    public static final double TankRatio = 10.;
    // TUNE!!
    public static final InvertedValue LeftInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RightInverted = InvertedValue.CounterClockwise_Positive;
    public static final double TankVelocityToleranceRPS = 0.;

    public static final double kDeadBand = 0.05; 
    public static final double kMaxRPS = 100.0; 

    // m/s control
    public static final double WheelDiameterMeters = 0.12; // TUNE!!
    public static final double WheelCircumferenceMeters = Math.PI * WheelDiameterMeters;
    public static final double kMaxSpeedMPS = 7.0; // TUNE!!
    public static final double TankVelocityToleranceMPS = 0.05;

    // TUNE!!
    public static final double kForwardMPS = 3.;
    public static final double kTurnMPS = 1.7;
    public static final double kInputPow = 3.;

    public static final double kArcadeSlewRateMPS = 0.5;
  }
}
