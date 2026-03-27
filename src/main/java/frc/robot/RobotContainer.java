// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand.ManualDriveCommand;
import frc.robot.commands.IntakerCommand.IntakeCommand;
import frc.robot.commands.IntakerCommand.OuttakeCommand;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Tank.TankSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  //private final TankSubsystem m_tank = new TankSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // private final CommandXboxController m_operatorController2 =
  //     new CommandXboxController(1);

  private final TankSubsystem m_tank = TankSubsystem.getInstance();
  private final ManualDriveCommand m_manualDrive;
  //private final IntakerSubsystem m_intake = IntakerSubsystem.getInstance();
  // private final frc.robot.subsystems.TankLike2910.TankSubsystem m_tank2910;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_tank2910 = null;
    
    // Configure the trigger bindings
    
  // Create a single ManualDriveCommand instance so we can call its methods from bindings
  this.m_manualDrive = new ManualDriveCommand(m_driverController, m_tank);
  m_tank.setDefaultCommand(this.m_manualDrive);

    // m_tank2910 = frc.robot.subsystems.TankLike2910.TankSubsystem.getInstance(m_driverController);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // m_driverController.b().onTrue(new InstantCommand(() -> m_tank.setRPSRight(2.0)));
    // m_driverController.b().onTrue(new InstantCommand(() -> m_tank.setRPSLeft(-2.0)));
    // m_driverController.a().onTrue(new InstantCommand(() -> m_tank.setRPSRight(0.0)));
    // m_driverController.a().onTrue(new InstantCommand(() -> m_tank.setRPSLeft(0.0)));

    // m_driverController.rightTrigger().whileTrue(new IntakeCommand(m_intake));

    // m_driverController.leftTrigger().whileTrue(new OuttakeCommand(m_intake));

    // m_driverController2.rightTrigger().whileTrue(new IntakeCommand(m_intake));
    // m_driverController2.leftTrigger().whileTrue(new OuttakeCommand(m_intake));

  m_driverController.b().onTrue(new InstantCommand(() -> m_tank.resetHeading()));
  m_driverController.x().onTrue(new InstantCommand(() -> m_manualDrive.switchDriveMode()));


  }

  /**
   * Autonomous 未配置，返回 null。
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
