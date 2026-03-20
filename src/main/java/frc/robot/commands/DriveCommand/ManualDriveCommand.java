// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommand;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.Tank.TankSubsystem;

public class ManualDriveCommand extends Command {
  private final TankSubsystem driveControl;
  private final CommandXboxController controller;
  private final DoubleSupplier leftStickX;
  private final DoubleSupplier leftStickY;
  private final DoubleSupplier rightStickX;
  private final DoubleSupplier rightStickY;

  public enum TankState {
    STOP,
    HEADLESS_TURN,
    HEADLESS_MOVE,
    ARCADE_DRIVE_MOVE,
    DRIVE_TO_POINT,
  }
  TankState state;
  
  Rotation2d currentHeading = new Rotation2d(0.);
  Pose2d targetPose = new Pose2d(0., 0., currentHeading);

  public ManualDriveCommand(CommandXboxController controller, TankSubsystem driveControl) {
    this.controller = controller;
    this.driveControl = driveControl;
    state = TankState.STOP;
    addRequirements(driveControl);

    leftStickX = () -> this.controller.getLeftX();
    leftStickY = () -> this.controller.getLeftY();
    rightStickX = () -> this.controller.getRightX();
    rightStickY = () -> this.controller.getRightY();

  }

  @Override
  public void initialize() {
    driveControl.resetHeading();
    state = TankState.STOP;
  }

  @Override
  public void execute() {

    double leftStickXIn = MathUtil.applyDeadband(leftStickX.getAsDouble(), TankConstants.K_DEADBAND);
    double leftStickYIn = MathUtil.applyDeadband(leftStickY.getAsDouble(), TankConstants.K_DEADBAND);
    double rightStickXIn = MathUtil.applyDeadband(rightStickX.getAsDouble(), TankConstants.K_DEADBAND);
    double rightStickYIn = MathUtil.applyDeadband(rightStickY.getAsDouble(), TankConstants.K_DEADBAND);




    switch (state) {
      case STOP:
        driveControl.stop();
        if ((leftStickXIn == 0) && (leftStickYIn == 0) && (rightStickXIn == 0) && (rightStickYIn == 0)){
          //driveControl.stop();
          break;
        } else {
          if(TankConstants.HeadlessControlConstants.USE_HEADLESS_CONTROL) {
            state = TankState.HEADLESS_TURN;
          } else {
            state = TankState.ARCADE_DRIVE_MOVE;
          }
        }
        break;
      case ARCADE_DRIVE_MOVE:
        arcadeDriveMove();
        break;
      case HEADLESS_TURN:
        headlessTurn();
        break;
      case HEADLESS_MOVE:
        headlessMove();
        break;
      case DRIVE_TO_POINT:
        driveToPoint(targetPose);
        break;
      default:
        driveControl.stop();
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    state = TankState.STOP;
    driveControl.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void arcadeDriveMove() {
    double forward = MathUtil.applyDeadband(controller.getLeftY(), TankConstants.K_DEADBAND);
    double turn = MathUtil.applyDeadband(controller.getRightX(), TankConstants.K_DEADBAND);
    driveControl.arcadedrive(forward, -turn);
  }

  private void headlessMove() {
    double leftX = MathUtil.applyDeadband(controller.getLeftX(), TankConstants.K_DEADBAND);
    double leftY = MathUtil.applyDeadband(controller.getLeftY(), TankConstants.K_DEADBAND);
    
    Translation2d stickVector = new Translation2d(leftX, leftY);
    
    if (stickVector.getNorm() > TankConstants.K_DEADBAND) {

  double stickAngle = MathUtil.inputModulus(stickVector.getAngle().getDegrees(), 0.0, 360.0);
      

  double currentHeading = driveControl.correctedHeadingDegrees();
  double angleDiff = MathUtil.inputModulus(stickAngle - currentHeading, 0, 360.0);
      //如果角度偏差大于5度，重新进入转向状态
      if (Math.abs(angleDiff) > 5.0) {
        state = TankState.HEADLESS_TURN;
        return;
      }

      driveControl.headlessMove(stickVector.getNorm());
    } else {

      state = TankState.STOP;
    }
  }

  private void headlessTurn() {
    double leftX = MathUtil.applyDeadband(controller.getLeftX(), TankConstants.K_DEADBAND);
    double leftY = MathUtil.applyDeadband(controller.getLeftY(), TankConstants.K_DEADBAND);
    double rightX = MathUtil.applyDeadband(controller.getRightX(), TankConstants.K_DEADBAND);
    double rightY = MathUtil.applyDeadband(controller.getRightY(), TankConstants.K_DEADBAND);
    
    Translation2d leftStickVector = new Translation2d(leftX, leftY);
    Translation2d rightStickVector = new Translation2d(rightX, rightY);
    
  double targetAngle;
    boolean hasValidInput = false;
    boolean isLeftStickInput = false;
    
    // 判断输入优先级和有效性
    if (leftStickVector.getNorm() > TankConstants.K_DEADBAND) {
      targetAngle = MathUtil.inputModulus(leftStickVector.getAngle().getDegrees(), 0.0, 360.0);
      hasValidInput = true;
      isLeftStickInput = true;
    } else if (rightStickVector.getNorm() > TankConstants.K_DEADBAND) {
      targetAngle = MathUtil.inputModulus(rightStickVector.getAngle().getDegrees(), 0.0, 360.0);
      hasValidInput = true;
      isLeftStickInput = false;
    } else {
      state = TankState.STOP;
      return;
    }
    
    if (hasValidInput) {
      boolean turnComplete = driveControl.headlessTurn(targetAngle, isLeftStickInput);
      
      if (turnComplete && isLeftStickInput) {
        state = TankState.HEADLESS_MOVE;
      }
    }
  }
  private void driveToPoint(Pose2d point) {
    driveControl.driveToPoint(point);
  }
}
