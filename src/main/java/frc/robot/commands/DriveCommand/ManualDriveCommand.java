// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.Tank.TankSubsystem;
import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;

public class ManualDriveCommand extends Command {
  private final TankSubsystem driveControl;
  private final CommandXboxController controller;
  private final DoubleSupplier leftStickXIn;
  private final DoubleSupplier leftStickYIn;
  private final DoubleSupplier rightStickXIn;
  private final DoubleSupplier rightStickYIn;

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

    leftStickXIn = () -> this.controller.getLeftX();
    leftStickYIn = () -> this.controller.getLeftY();
    rightStickXIn = () -> this.controller.getRightX();
    rightStickYIn = () -> this.controller.getRightY();
  }

  @Override
  public void initialize() {
    state = TankState.STOP;
  }

  @Override
  public void execute() {

    double leftStickX =
        MathUtil.applyDeadband(leftStickXIn.getAsDouble(), TankConstants.K_DEADBAND);
    double leftStickY =
        MathUtil.applyDeadband(leftStickYIn.getAsDouble(), TankConstants.K_DEADBAND);
    double rightStickX =
        MathUtil.applyDeadband(rightStickXIn.getAsDouble(), TankConstants.K_DEADBAND);
    double rightStickY =
        MathUtil.applyDeadband(rightStickYIn.getAsDouble(), TankConstants.K_DEADBAND);

    switch (state) {
      case STOP:
        driveControl.stop();
        if ((leftStickX == 0) && (leftStickY == 0) && (rightStickX == 0) && (rightStickY == 0)) {
          // driveControl.stop();
          break;
        } else {
          if (TankConstants.HeadlessControlConstants.USE_HEADLESS_CONTROL) {
            state = TankState.HEADLESS_TURN;
          } else {
            state = TankState.ARCADE_DRIVE_MOVE;
          }
        }
        break;
      case ARCADE_DRIVE_MOVE:
        arcadeDriveMove(leftStickX, rightStickY);
        break;
      case HEADLESS_TURN:
        headlessTurn(leftStickX, leftStickY, rightStickX, rightStickY);
        break;
      case HEADLESS_MOVE:
        headlessMove(leftStickX, leftStickY);
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

  private void arcadeDriveMove(double forwardIn, double turnIn) {
    double forward = forwardIn;
    double turn = turnIn;
    driveControl.arcadedrive(forward, -turn);
  }

  private void headlessMove(double leftStickX, double leftStickY) {
    double leftX = leftStickX;
    double leftY = leftStickY;

    Translation2d stickVector = new Translation2d(leftX, leftY);

    double stickAngle = MathUtil.inputModulus(stickVector.getAngle().getDegrees() - 90.0, 0.0, 360.0);

    double currentHeading = driveControl.getHeading();
    double angleDiff = stickAngle - currentHeading;
    if (Math.abs(angleDiff) > 5.0) {
      state = TankState.HEADLESS_TURN;
      return;
    }

    double forward = Math.pow(stickVector.getNorm(), 3.);

    driveControl.headlessMove(forward);
  }

  private void headlessTurn(double leftStickX, double leftStickY, double rightStickX, double rightStickY) {
    double leftX = leftStickX;
    double leftY = leftStickY;
    double rightX = rightStickX;
    double rightY = rightStickY;

    Translation2d leftStickVector = new Translation2d(leftX, leftY);
    Translation2d rightStickVector = new Translation2d(rightX, rightY);

    double targetAngle;
    boolean hasValidInput = false;
    boolean isLeftStickInput = false;

    if (leftStickVector.getNorm() > TankConstants.K_DEADBAND) {
      targetAngle = MathUtil.inputModulus(leftStickVector.getAngle().getDegrees() - 90., 0.0, 360.0);
      isLeftStickInput = true;
    } else if (rightStickVector.getNorm() > TankConstants.K_DEADBAND) {
      targetAngle = MathUtil.inputModulus(driveControl.getHeading() + Math.pow(rightY, 3.) * .5, 0.0, 360.0);
      isLeftStickInput = false;
    } else {
      state = TankState.STOP;
      return;
    }
      boolean turnComplete = driveControl.headlessTurn(targetAngle, isLeftStickInput);
      if (turnComplete && isLeftStickInput) {
        state = TankState.HEADLESS_MOVE;
      }
    }
  

  private void driveToPoint(Pose2d point) {
    driveControl.driveToPoint(point);
  }
}
