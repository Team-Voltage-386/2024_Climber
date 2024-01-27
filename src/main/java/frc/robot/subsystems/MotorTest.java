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
import frc.robot.commands.AGoToPositionCommand;

public class MotorTest extends SubsystemBase {

    //private CANSparkMax motorB = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax motorA = new CANSparkMax(11, MotorType.kBrushless);

    private double oldA = 0, oldB = 0;
    private boolean oldIsOn = false;
    private double oldBothMotor = 0;
    private boolean oldUseBothMotor;
    private boolean oldInvertBothMotor;
    private ShuffleboardTab motorTab;

    private SimpleWidget motorOnOff;
    private SimpleWidget motorAShuffable, motorBShuffable;
    private SimpleWidget bothMotorShuffable;
    private SimpleWidget useBothMotor;
    private SimpleWidget invertBothMotor;
    private SimpleWidget motorARPM;
    private SimpleWidget motorBRPM;

    private SimpleWidget motorAEnc;
    private SimpleWidget motorAAbsRPM;
    private SimpleWidget motorARelEnc;
    private SimpleWidget motorBEnc;

    private SendableChooser<Command> motorASetPosition;

    public MotorTest() {
        motorTab = Shuffleboard.getTab("Motor Controls");
        motorOnOff = motorTab.add("Are Motors On", false);
        motorAShuffable = motorTab.add("Motor A Percent", 0);
        motorBShuffable = motorTab.add("Motor B Percent", 0);
        bothMotorShuffable = motorTab.add("Set Both Motors", 0);
        useBothMotor = motorTab.add("Use Both Motor Setting", false);
        invertBothMotor = motorTab.add("Invert Both Motor Setting", false);
        motorARPM = motorTab.add("Motor A RPM", 0.0);
        motorBRPM = motorTab.add("Motor B RPM", 0.0);

        motorAEnc = motorTab.add("Motor A ENC", 0.0);
        motorAAbsRPM = motorTab.add("Motor A ABS RMP", 0.0);
        motorARelEnc = motorTab.add("Motor A Rel ENC", 0.0);
        motorBEnc = motorTab.add("Motor B ENC", 0.0);

        motorASetPosition = new SendableChooser<Command>();
        motorASetPosition.addOption("Up", this.aGoToPositionCommand(20.0));
        motorASetPosition.addOption("Down", this.aGoToPositionCommand(3.0));
        motorTab.add("Motor A Choices", motorASetPosition);
    }

    public static enum Direction {
        UP,
        DOWN
    };

    public double getRelPosA() {
        RelativeEncoder relEncA = motorA.getEncoder();
        return relEncA.getPosition();
    }

    public double adjustSpeedForDirection(double speed, Direction direction) {
        switch(direction) {
            case DOWN: {
                return -1 * speed;
            }
            default: {
                return speed;
            }
        }
    }

    public void setMotorASpeed(double speed) {
        motorA.set(speed);
    }

    public Command aGoToPositionCommand(double position) {
        return new AGoToPositionCommand(this, position, 0.05);
    }

    public Command getSelected() {
        return this.motorASetPosition.getSelected();
    }

    @Override
    public void periodic() {
        RelativeEncoder relEncA = motorA.getEncoder();
        SparkAbsoluteEncoder absEncA = motorA.getAbsoluteEncoder(Type.kDutyCycle);
        this.motorAEnc.getEntry().setDouble(Math.abs(absEncA.getPosition()));
        this.motorAAbsRPM.getEntry().setDouble(Math.abs(absEncA.getVelocity()));
        this.motorARelEnc.getEntry().setDouble(Math.abs(relEncA.getPosition()));
        this.motorARPM.getEntry().setDouble(Math.abs(relEncA.getVelocity()));

        // oldUseBothMotor = useBothMotor.getEntry().getBoolean(false);
        // oldInvertBothMotor = invertBothMotor.getEntry().getBoolean(false);

        // oldBothMotor = bothMotorShuffable.getEntry().getDouble(0);
        // if (oldBothMotor > 100) {
        //     oldBothMotor = 100;
        // } else if (oldBothMotor < 0) {
        //     oldBothMotor = 0;
        // }

        // double A, B;
        // if (oldUseBothMotor) {
        //     if (oldInvertBothMotor) {
        //         A = oldBothMotor;
        //         B = -1 * oldBothMotor;
        //     } else {
        //         A = -1 * oldBothMotor;
        //         B = oldBothMotor;
        //     }
        // } else {
        //     A = motorAShuffable.getEntry().getDouble(0);
        //     B = motorBShuffable.getEntry().getDouble(0);
        // }

        // if (A > 100) {
        //     A = 100;
        // } else if (A < -100) {
        //     A = -100;
        // }

        // if (B > 100) {
        //     B = 100;
        // } else if (B < -100) {
        //     B = -100;
        // }

        // oldA = A;
        // oldB = B;

        // boolean isOn = motorOnOff.getEntry().getBoolean(false);

        // if (isOn) {
        //     if (!oldIsOn) {
        //         // Asked to turn on
        //         oldIsOn = true;
        //         motorA.set(oldA / 100);
        //         motorB.set(oldB / 100);
        //     }
        // } else {
        //     if (oldIsOn) {
        //         // Asked to turn off
        //         oldIsOn = false;
        //         motorA.set(0);
        //         motorB.set(0);
        //     }
        // }

        

        // RelativeEncoder absEncB = motorB.getEncoder();
        // this.motorBRPM.getEntry().setDouble(Math.abs(absEncB.getVelocity()));
    }
}