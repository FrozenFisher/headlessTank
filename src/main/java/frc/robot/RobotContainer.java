// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand.ManualDriveCommand;
import frc.robot.subsystems.Tank.TankSubsystem;

public class RobotContainer {
  // private final TankSubsystem m_tank = new TankSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // private final CommandXboxController m_operatorController2 =
  //     new CommandXboxController(1);

  private final TankSubsystem m_tank = TankSubsystem.getInstance();

  // private final IntakerSubsystem m_intake = IntakerSubsystem.getInstance();
  // private final frc.robot.subsystems.TankLike2910.TankSubsystem m_tank2910;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_tank2910 = null;

    // Configure the trigger bindings

    // Create a single ManualDriveCommand instance so we can call its methods from bindings

    m_tank.setDefaultCommand(
        new ManualDriveCommand(
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getRightX(),
            () -> m_driverController.getRightY(),
            m_tank));
            
  }

  private void configureBindings() {

    // m_driverController.b().onTrue(new InstantCommand(() -> m_tank.setRPSRight(2.0)));
    // m_driverController.b().onTrue(new InstantCommand(() -> m_tank.setRPSLeft(-2.0)));
    // m_driverController.a().onTrue(new InstantCommand(() -> m_tank.setRPSRight(0.0)));
    // m_driverController.a().onTrue(new InstantCommand(() -> m_tank.setRPSLeft(0.0)));

    // m_driverController.rightTrigger().whileTrue(new IntakeCommand(m_intake));

    // m_driverController.leftTrigger().whileTrue(new OuttakeCommand(m_intake));

    // m_driverController2.rightTrigger().whileTrue(new IntakeCommand(m_intake));
    // m_driverController2.leftTrigger().whileTrue(new OuttakeCommand(m_intake));
  }

  /** Autonomous 未配置，返回 null。 */
  public Command getAutonomousCommand() {
    return null;
  }
}
