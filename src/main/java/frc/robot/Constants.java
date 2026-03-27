// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

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
    public static final int TANK_LEFT_MOTOR_ID = 2;
    public static final int TANK_RIGHT_MOTOR_ID = 1;
    public static final int TANK_PIGEON_ID = 3; // PigeonID
    
    public static final double K_P = 0.1;//TUNE!!
    public static final double K_I = 0.;
    public static final double K_D = 0.;
    public static final double K_V = 1.3;//TUNE!!
    public static final double K_S = 0.254;
    public static final double TANK_RATIO = 10.;
    //TUNE!!若负责移动的电机有传动系统，新增对应传动比
    public static final InvertedValue LEFT_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RIGHT_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final double TANK_VELOCITY_TOLERANCE_RPS = 0.;

    public static final double K_DEADBAND = 0.05; // 手柄死区
    public static final double K_MAX_RPS = 300.0; // 输出RPS限幅

    //m/s control
    public static final double WHEEL_DIAMETER_METERS = 0.12; //TUNE!!轮径
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    public static final double K_MAX_SPEED_MPS = 7.0; //TUNE!!最大线速度
    public static final double TANK_VELOCITY_TOLERANCE_MPS = 0.05; // 速度到达容差(m/s)

    //TUNE!!手柄到目标速度(m/s)比例（前进与转向分量）
    public static final double K_FWD_MPS = 3.;
    public static final double K_TURN_MPS = 1.7;
    public static final double K_INPUT_POW = 3.;

    public static final boolean USE_TANK_2910 = false; // 切换到TankLike2910风格

    public static class HeadlessControlConstants {

      public static final double TURN_K_P = 0.022;//TUNE!!
      public static final double TURN_K_I = 0.;
      public static final double TURN_K_D = 0.0;
      // If your IMU (Pigeon) is mounted rotated, adjust this so headings reported
      // by getHeading() align with the field frame used by the rest of the code.
      // Positive values rotate the reported heading counter-clockwise.
      public static final double HEADING_OFFSET_DEGREES = 0.0; // TUNE this (default assumes sensor is +90deg rotated)
    }
  }
  public static class LimelightConstants {
    public static final int PIPELINE = 0;
    
  }
  public static class IntakerConstants {
    public static final int INTAKE_MOTOR_ID = 4;
    public static final double INTAKE_RATIO = 1.0;
    public static final double K_P = 0.3;//TUNE!!
    public static final double K_I = 0.;
    public static final double K_D = 0.;
    public static final double K_V = 0.09;//TUNE!!
    public static final double K_S = 0.24;
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double INTAKE_VELOCITY_TOLERANCE_RPS = 0.;
    public static final double INTAKE_MAX_RPS = 30.0;

    
    public static final double INTAKE_HIGH_RPS =-20.0; // Intake RPS
    public static final double INTAKE_LOW_RPS =-5.0; // Intake RPS
    public static final double OUTTAKE_HIGH_RPS = 40.0; 
    public static final double OUTTAKE_LOW_RPS = 10.0; 
    public static final double INTAKE_HIGH_DURATION = 1.0; 
    public static final double OUTTAKE_HIGH_DURATION = 1.0; 

  }
}
