package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.MotorGoToPositionCommand;
import frc.robot.components.Motor;

public class MotorTest extends SubsystemBase {
    private Motor m_motorA;
    private Motor m_motorB;

    private ShuffleboardTab motorTab;

    private SimpleWidget motorARPM;
    private SimpleWidget motorARelEnc;
    private SimpleWidget motorBRelEnc;

    private SendableChooser<Command> motorASetPosition;
    private SendableChooser<Command> motorBSetPosition;

    public MotorTest() {
        m_motorA = new Motor(11);
        m_motorB = new Motor(12);

        motorTab = Shuffleboard.getTab("Motor Controls");
        motorARPM = motorTab.add("Motor A RPM", 0.0);

        motorARelEnc = motorTab.add("Motor A Rel ENC", 0.0);

        motorASetPosition = new SendableChooser<Command>();
        motorASetPosition.addOption("Up", this.tiltClimberCommand(this.m_motorA, 33));
        motorASetPosition.addOption("Down", this.tiltClimberCommand(this.m_motorA, 4.5));
        motorTab.add("Motor A Choices", motorASetPosition);

        motorBSetPosition = new SendableChooser<Command>();
        motorBSetPosition.addOption("Up", this.tiltClimberCommand(this.m_motorB, 230));
        motorBSetPosition.addOption("Down", this.tiltClimberCommand(this.m_motorB, -40));
        motorTab.add("Motor B Choices", motorBSetPosition);

        motorBRelEnc = motorTab.add("Motor B Rel ENC", 0.0);
    }

    public static enum Direction {
        UP,
        DOWN
    };

    public double adjustSpeedForDirection(Motor motor, double speed, Direction direction) {
        switch(direction) {
            case DOWN: {
                return motor.adjustSpeedForDirection(speed, Motor.Direction.COUNTERCLOCKWISE);
            }
            default: {
                return motor.adjustSpeedForDirection(speed, Motor.Direction.CLOCKWISE);
            }
        }
    }

    public Command tiltClimberCommand(Motor motor, double position) {
        return new MotorGoToPositionCommand(this, motor, position, 0.3);
    }

    public Command getSelected() {
        return this.motorBSetPosition.getSelected();
    }

    @Override
    public void periodic() {
        this.motorARelEnc.getEntry().setDouble(m_motorA.getRelativePosition());
        this.motorARPM.getEntry().setDouble(m_motorA.getSpeed());
        this.motorBRelEnc.getEntry().setDouble(m_motorB.getRelativePosition());

    }
}