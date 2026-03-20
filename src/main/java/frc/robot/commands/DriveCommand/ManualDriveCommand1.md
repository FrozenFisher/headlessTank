// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommand;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.Tank.TankSubsystem;

public class ManualDriveCommand extends Command {
  private final TankSubsystem driveControl;
  private final DoubleSupplier leftStickX;
  private final DoubleSupplier leftStickY;
  private final DoubleSupplier rightStickX;
  private final DoubleSupplier rightStickY;


  public ManualDriveCommand(DoubleSupplier leftStickX, DoubleSupplier leftStickY, DoubleSupplier rightStickX, DoubleSupplier rightStickY, TankSubsystem driveControl) {
    this.driveControl = driveControl;
    this.leftStickX = leftStickX;
    this.leftStickY = leftStickY;
    this.rightStickX = rightStickX;
    this.rightStickY = rightStickY;
    addRequirements(driveControl);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    double leftStickXIn = MathUtil.applyDeadband(leftStickX.getAsDouble(), TankConstants.K_DEADBAND);
    double leftStickYIn = MathUtil.applyDeadband(leftStickY.getAsDouble(), TankConstants.K_DEADBAND);
    double rightStickXIn = MathUtil.applyDeadband(rightStickX.getAsDouble(), TankConstants.K_DEADBAND);
    double rightStickYIn = MathUtil.applyDeadband(rightStickY.getAsDouble(), TankConstants.K_DEADBAND);
    double forward = Math.signum(leftStickYIn) * Math.pow(Math.abs(leftStickYIn), TankConstants.K_INPUT_POW);
    double turn = Math.signum(rightStickXIn) * Math.pow(Math.abs(rightStickXIn), TankConstants.K_INPUT_POW);
    driveControl.arcadedrive(forward, -turn);
  }

  @Override
  public void end(boolean interrupted) {
    driveControl.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
