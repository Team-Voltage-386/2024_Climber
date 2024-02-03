package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.commands.ElevatorGoToPositionCommand;
import frc.robot.components.ElevatorDirection;
import frc.robot.components.Motor;

public class MotorTest extends SubsystemBase {
    private static final String MOTOR_A_NAME = "A";
    private static final String MOTOR_B_NAME = "B";

    private ShuffleboardTab motorTab;

    private Motor m_motorA;
    private Motor m_motorB;
    private Command m_motorACurrentCommand;
    private Command m_motorBCurrentCommand;

    private SendableChooser<Command> elevatorSetPosition;

    public MotorTest() {
        motorTab = Shuffleboard.getTab("Motor Controls");

        final double kp = 0.00;
        final double ks = 0.00;
        final double kv = 0.00;
        m_motorA = new Motor(motorTab, MOTOR_A_NAME, MotorConstants.kDeviceIDMotorA,
            new ProfiledPIDController(
                kp,
                0,
                0,
                new TrapezoidProfile.Constraints(
                        1142,
                        100
                )
            ),
            new SimpleMotorFeedforward(
                ks,
                kv
            )
        );
        m_motorACurrentCommand = null;
        m_motorB = new Motor(motorTab, MOTOR_B_NAME, MotorConstants.kDeviceIDMotorB,
            new ProfiledPIDController(
                kp,
                0,
                0,
                new TrapezoidProfile.Constraints(
                        1142,
                        100
                )
            ),
            new SimpleMotorFeedforward(
                ks,
                kv
            )
        );
        m_motorBCurrentCommand = null;

        elevatorSetPosition = new SendableChooser<Command>();
        elevatorSetPosition.addOption("Up", this.elevatorClimberCommand(this.m_motorA, MotorConstants.kMaxElevatorUpRelativeEncoderPositionUp).alongWith(this.elevatorClimberCommand(this.m_motorB, MotorConstants.kMaxElevatorUpRelativeEncoderPositionUp)));
        elevatorSetPosition.addOption("Down", this.elevatorClimberCommand(this.m_motorA, MotorConstants.kMinElevatorDownRelativeEncoderPositionDown).alongWith(this.elevatorClimberCommand(this.m_motorB, MotorConstants.kMinElevatorDownRelativeEncoderPositionDown)));
        motorTab.add("Elevator Choices", elevatorSetPosition);
    }

    public double adjustSpeedForDirection(Motor motor, double speed, ElevatorDirection direction) {
        switch(direction) {
            case DOWN: {
                return motor.adjustSpeedForDirection(speed, Motor.Direction.COUNTERCLOCKWISE);
            }
            default: {
                return motor.adjustSpeedForDirection(speed, Motor.Direction.CLOCKWISE);
            }
        }
    }

    public Command elevatorClimberCommand(Motor motor, double position) {
        if (motor.getName() == MOTOR_A_NAME) {
            if (this.m_motorACurrentCommand != null) {
                this.m_motorACurrentCommand.cancel();
            }
            this.m_motorACurrentCommand = new ElevatorGoToPositionCommand(this, motor, position);
            return this.m_motorACurrentCommand;
        } else if (motor.getName() == MOTOR_B_NAME) {
            if (this.m_motorBCurrentCommand != null) {
                this.m_motorBCurrentCommand.cancel();
            }
            this.m_motorBCurrentCommand = new ElevatorGoToPositionCommand(this, motor, position);
            return this.m_motorBCurrentCommand;
        } else {
            return null;
        }
    }

    public Command getSelected() {
        return this.elevatorSetPosition.getSelected();
    }

    @Override
    public void periodic() {
    }
}