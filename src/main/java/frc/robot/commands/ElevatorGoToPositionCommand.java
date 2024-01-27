// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.components.ElevatorDirection;
import frc.robot.components.Motor;
import frc.robot.components.MotorGoToPositionInfo;
import frc.robot.subsystems.MotorTest;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorGoToPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorTest m_subsystem;
  private final Motor m_motor;
  private final double m_targetPosition;
  private ElevatorDirection m_direction;
  private MotorGoToPositionInfo m_goToPositionInfo;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorGoToPositionCommand(MotorTest subsystem, Motor motor, double position) {
    m_subsystem = subsystem;
    m_motor = motor;
    m_targetPosition = position;
    m_direction = this.getDirection();
    m_goToPositionInfo = new MotorGoToPositionInfo(0.0, Timer.getFPGATimestamp());
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  public ElevatorDirection getDirection() {
    double currentPosition = this.m_motor.getRelativePosition();
    System.out.printf("CURRENT POS: %f, TARGET POS: %f. CURRENT < TARGET: %b\n", currentPosition, m_targetPosition, currentPosition < m_targetPosition);
    if (currentPosition < m_targetPosition) {
      return ElevatorDirection.UP;
    } else {
      return ElevatorDirection.DOWN;
    }
  }

  public double getTarget() {
    return this.m_targetPosition;
  }

  public boolean reachedPos() {
    double currentPos = this.m_motor.getRelativePosition();
    switch (this.m_direction) {
      case UP: {
        return currentPos > m_targetPosition || Math.abs(currentPos - m_targetPosition) < 0.1;
      }
      case DOWN: {
        return currentPos < m_targetPosition || Math.abs(currentPos - m_targetPosition) < 0.1;
      }
      default: {
        // Unrecognized option
        return true;
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_direction = this.getDirection();
    m_goToPositionInfo = new MotorGoToPositionInfo(0.0, Timer.getFPGATimestamp());

    switch (this.m_direction) {
      case UP: {
        System.out.println("UP");
        break;
      }
      case DOWN: {
        System.out.println("DOWN");
        break;
      }
      default: {
        System.out.println("UNKNOWN");
        break;
      }
    }
    // if (!this.reachedPos()) {
    //   this.m_motor.setSpeed(this.m_subsystem.adjustSpeedForDirection(this.m_motor, 0.2, this.m_direction));
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_motor.goToPosition(m_targetPosition, m_goToPositionInfo, m_subsystem.adjustSpeedForDirection(m_motor, 1.0, m_direction));
    this.m_motor.updateWidgets(m_targetPosition, m_goToPositionInfo);
  }

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
