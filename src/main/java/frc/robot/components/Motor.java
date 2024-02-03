package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkAnalogSensor.Mode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class Motor {
    private CANSparkMax m_motor;
    private final ProfiledPIDController m_PIDController;
    private final SimpleMotorFeedforward m_feedforward;
    private final String m_motorName;
    private SimpleWidget m_currentVelocityWidget;
    private SimpleWidget m_targetVelocityWidget;
    private SimpleWidget m_pidValWidget;
    private SimpleWidget m_feedForwardValWidget;
    private SimpleWidget m_outputCurrent;
    private SimpleWidget m_pidSetpointSpeed;
    private SimpleWidget m_pidSetpointPosition;
    private SimpleWidget m_analogSensorPosition;
    private SimpleWidget m_relativeEncoderPosition;

    public Motor(ShuffleboardTab motorTab, String motorName, int deviceId, ProfiledPIDController PIDController, SimpleMotorFeedforward feedforward) {
        m_motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        m_motor.setSmartCurrentLimit(40);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_PIDController = PIDController;
        m_feedforward = feedforward;
        m_motorName = motorName;
        m_currentVelocityWidget = motorTab.add("Motor " + m_motorName + " Current Velocity", 0.0);
        m_targetVelocityWidget = motorTab.add("Motor " + m_motorName + " Target Velocity", 0.0);
        m_pidValWidget = motorTab.add("Motor " + m_motorName + " PID", 0.0);
        m_feedForwardValWidget = motorTab.add("Motor " + m_motorName + " Feed Forward", 0.0);
        m_outputCurrent = motorTab.add("Motor " + m_motorName + " Output Current", 0.0);
        m_pidSetpointSpeed = motorTab.add("Motor " + m_motorName + " PID Setpoint Speed", 0.0);
        m_pidSetpointPosition = motorTab.add("Motor " + m_motorName + " PID Setpoint Position", 0.0);
        m_analogSensorPosition = motorTab.add("Motor " + m_motorName + " Analog Sensor Position", 0.0);
        m_relativeEncoderPosition = motorTab.add("Motor " + m_motorName + " Rel ENC Position", 0.0);
    }

    public static enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE
    };

    public String getName() {
        return this.m_motorName;
    }

    public double getRelativePosition() {
        RelativeEncoder relEnc = m_motor.getEncoder();
        return relEnc.getPosition();
    }

    public double adjustSpeedForDirection(double speed, Direction direction) {
        switch(direction) {
            case COUNTERCLOCKWISE: {
                return -1 * speed;
            }
            default: {
                return speed;
            }
        }
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        this.m_motor.stopMotor();
    }

    public double getSpeed() {
        RelativeEncoder relEnc = m_motor.getEncoder();
        return relEnc.getVelocity();
    }

    public double getVoltage() {
        return m_motor.getBusVoltage();
    }

    public double getTargetVelocity() {
        return m_PIDController.getSetpoint().velocity;
    }

    public double getPidVal(double goalPosition) {
        return m_PIDController.calculate(this.getRelativePosition(), goalPosition);
    }

    public double getFFVal(MotorGoToPositionInfo goToPositionInfo) {
        double targetVelocity = m_PIDController.getSetpoint().velocity;
        double acceleration = (targetVelocity - goToPositionInfo.getLastSpeed())
                / (Timer.getFPGATimestamp() - goToPositionInfo.getLastTime());
        return m_feedforward.calculate(m_PIDController.getSetpoint().velocity, acceleration);
    }

    // Controls a simple motor's position using a SimpleMotorFeedforward
    // and a ProfiledPIDController
    public void goToPosition(double goalPosition, MotorGoToPositionInfo goToPositionInfo, double adjuster) {
        double targetVelocity = m_PIDController.getSetpoint().velocity;
        double acceleration = (targetVelocity - goToPositionInfo.getLastSpeed())
                / (Timer.getFPGATimestamp() - goToPositionInfo.getLastTime());

        double actualVelocity = m_motor.getEncoder().getVelocity();

        double pidVal = m_PIDController.calculate(this.getRelativePosition(), goalPosition);
        double FFVal = m_feedforward.calculate(m_PIDController.getSetpoint().velocity, acceleration);

        this.m_motor.setVoltage(adjuster * (pidVal + FFVal));
        goToPositionInfo.setLastSpeed(actualVelocity);
        goToPositionInfo.setLastTime(Timer.getFPGATimestamp());
    }

    public void updateWidgets(double goalPosition, MotorGoToPositionInfo goToPositionInfo) {
        this.m_currentVelocityWidget.getEntry().setDouble(this.getSpeed());
        this.m_targetVelocityWidget.getEntry().setDouble(this.getTargetVelocity());
        this.m_outputCurrent.getEntry().setDouble(this.m_motor.getOutputCurrent());
        this.m_pidSetpointSpeed.getEntry().setDouble(m_PIDController.getSetpoint().velocity);
        this.m_pidSetpointPosition.getEntry().setDouble(m_PIDController.getSetpoint().position);
        this.m_pidValWidget.getEntry().setDouble(this.getPidVal(goalPosition));
        this.m_feedForwardValWidget.getEntry().setDouble(this.getFFVal(goToPositionInfo));

        SparkAnalogSensor analogSensor = this.m_motor.getAnalog(Mode.kAbsolute);
        this.m_analogSensorPosition.getEntry().setDouble(analogSensor.getPosition());

        this.m_relativeEncoderPosition.getEntry().setDouble(this.getRelativePosition());
    }
}
