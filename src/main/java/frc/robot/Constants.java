// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import dev.doglog.DogLog;


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
    public static final double kForwardMPS = 1.5;
    public static final double kTurnMPS = 0.4;
    public static final double kInputPow = 1.;

    public static final double kArcadeSlewRateMPS = 0.8;
  }


  public static class Ports {
    public static class LED {
      public static final int LED_PWM_PORT = 9;
    }
  }

  public static class Settings {
    public static class LED {
      public static final int LED_LENGTH = 60;
      public static final int[] TANK_BUFFER = {0, 60};

    }

    public interface EnabledSubsystems {

          BooleanSubscriber FEEDER = DogLog.tunable("Enabled Subsystems/Feeder", false);

          BooleanSubscriber INTAKE = DogLog.tunable("Enabled Subsystems/Intake", true);

          BooleanSubscriber LED = DogLog.tunable("Enabled Subsystems/LED", false);

          BooleanSubscriber HANDOFF = DogLog.tunable("Enabled Subsystems/Handoff", false);

          BooleanSubscriber SHOOTER = DogLog.tunable("Enabled Subsystems/Shooter", false);

          BooleanSubscriber VISION = DogLog.tunable("Enabled Subsystems/Vision", false);

          BooleanSubscriber SWERVE = DogLog.tunable("Enabled Subsystems/Swerve", false);
        }
      
        public interface LEDs {

        // TODO: Get actual length of led, along with length of individual sections
        int LED_LENGTH = 60;

        // Buffer Views {Starting Index, Ending Index}
        

        // LED Pattern
        
        LEDPattern DISABLED = LEDPattern.solid(Color.kPurple);
        LEDPattern TURNING_LEFT = LEDPattern.rainbow(255, 255);
        LEDPattern TURNING_RIGHT = LEDPattern.rainbow(255, 255);
        LEDPattern FORWARD =  LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue);
        LEDPattern STOPPED = LEDPattern.solid(Color.kPurple);
    }

  }


}
