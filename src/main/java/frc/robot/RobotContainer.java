// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TrapSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.TrapInCommand;
import frc.robot.commands.TrapManualInCommand;
import frc.robot.commands.TrapManualOutCommand;
import frc.robot.commands.TrapOutCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TrapSubsystem m_trapSubsystem = new TrapSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  // public MotorTest getMotorTest() {
  // return this.m_motors;
  // }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController.b().whileTrue(new TrapOutCommand(m_trapSubsystem));
    // m_driverController.x().whileTrue(new TrapInCommand(m_trapSubsystem));
    // m_driverController.y().whileTrue(new ElevatorUpCommand(m_ElevatorSubsystem));
    // m_driverController.a().whileTrue(new
    // ElevatorDownCommand(m_ElevatorSubsystem));

    m_driverController.povRight().whileTrue(new TrapManualOutCommand(m_trapSubsystem));
    m_driverController.povLeft().whileTrue(new TrapManualInCommand(m_trapSubsystem));
    m_driverController.povUp().onTrue(new TrapOutCommand(m_trapSubsystem));
    m_driverController.povDown().onTrue(new TrapInCommand(m_trapSubsystem));

    m_driverController.axisGreaterThan(2, 0.5).whileTrue(new ElevatorDownCommand(m_ElevatorSubsystem));
    m_driverController.axisGreaterThan(3, 0.5).whileTrue(new ElevatorUpCommand(m_ElevatorSubsystem));

    m_driverController.rightBumper().whileTrue(Commands.runOnce(() -> m_trapSubsystem.setTrapIntakeMotorOn()))
        .onFalse(Commands.runOnce(() -> m_trapSubsystem.setTrapIntakeMotorOff()));
    m_driverController.leftBumper().whileTrue(Commands.runOnce(() -> m_trapSubsystem.setTrapIntakeMotorReverse()))
        .onFalse(Commands.runOnce(() -> m_trapSubsystem.setTrapIntakeMotorOff()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}

// change notes:
// when flipping between states a lot, the modes do not switch cleanly. Going
// down to the bottom is not always resetting to zero, those things should
// pretty much be tied together