package frc.robot.components;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MotorGoToPositionCommand;

public class Motor {
    private CANSparkMax m_motor;


    public Motor(int deviceId) {
        m_motor = new CANSparkMax(deviceId, MotorType.kBrushless);
    }

    public static enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE
    };

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

    public double getSpeed() {
        RelativeEncoder relEnc = m_motor.getEncoder();
        return relEnc.getVelocity();
    }
}
