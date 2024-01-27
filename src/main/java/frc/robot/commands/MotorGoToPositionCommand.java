// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.components.Motor;
import frc.robot.subsystems.MotorTest;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MotorGoToPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorTest m_subsystem;
  private final Motor m_motor;
  private final double m_targetPosition;
  private final double m_maxSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MotorGoToPositionCommand(MotorTest subsystem, Motor motor, double position, double maxSpeed) {
    m_subsystem = subsystem;
    m_motor = motor;
    m_targetPosition = position;
    m_maxSpeed = maxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public MotorTest.Direction getDirection() {
    double currentPos = this.m_motor.getRelativePosition();
    if (currentPos < m_targetPosition) {
      return MotorTest.Direction.UP;
    } else {
      return MotorTest.Direction.DOWN;
    }
  }

  public boolean reachedPos() {
    double currentPos = this.m_motor.getRelativePosition();
    return Math.abs(currentPos - m_targetPosition) < 0.1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!this.reachedPos()) {
      this.m_motor.setSpeed(this.m_subsystem.adjustSpeedForDirection(this.m_motor, this.m_maxSpeed, this.getDirection()));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("STOPPING");
    this.m_motor.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.reachedPos();
  }
}
