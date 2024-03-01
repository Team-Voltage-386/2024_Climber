package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.commands.TrapExtendCommand;

public class TrapSubsystem extends SubsystemBase {
    // constants
    private int m_extendLimitDIOChannel = 1;
    private int kTrapExtendMotorID = 1;

    private PowerDistribution m_PDH = new PowerDistribution(1, ModuleType.kRev);

    // Reflective sensor
    private DigitalInput m_extendLimit = new DigitalInput(m_extendLimitDIOChannel);

    // During testing, the only speed that we needed to go at was 100%
    private double m_motorPercentage = 1;

    private TalonSRX m_trapExtendMotor;
    private TalonSRX m_trapIntakeMotor;

    private double kHoldingPieceCurrent = 10; // Figure out during testing. Should be the current drop when the intake
                                              // motor is holding a piece all the way back
    private double kMaxExtendInCurrentLimit = 10; // Need to figure out during testing. Try to read the current spike
                                                  // when the motor starts stalling becuase the extension is at the
                                                  // maximum distance in

    private ShuffleboardTab m_trapSubsystemTab;
    private SimpleWidget m_motorPercentageEntry;
    private SimpleWidget m_isLimitTriggeredEntry;

    // private color sensor

    public TrapSubsystem() {
        // m_trapExtendMotor = new TalonSRX(MotorConstants.kTrapExtendMotorID);
        m_trapExtendMotor = new TalonSRX(MotorConstants.kTrapExtendMotorID);
        m_trapExtendMotor.setInverted(false);
        m_trapIntakeMotor = new TalonSRX(MotorConstants.kTrapIntakeMotorID);
        m_trapIntakeMotor.setInverted(false);

        m_trapSubsystemTab = Shuffleboard.getTab("Trap Subsystem");
        m_motorPercentageEntry = m_trapSubsystemTab.add("Extend Motor Current", 0.0);
        m_isLimitTriggeredEntry = m_trapSubsystemTab.add("Is Limit Triggered?", false);

    }

    public void setTrapExtendMotor(double motorPercentage) {
        m_trapExtendMotor.set(TalonSRXControlMode.PercentOutput, motorPercentage);
    }

    public boolean isLimitTriggered() {
        return m_extendLimit.get();
    }

    public Command trapExtendCommand(double motorPercentage) {
        return new TrapExtendCommand(motorPercentage, this);
    }

    public void setTrapIntakeMotor(double motorPercentage) {
        m_trapIntakeMotor.set(TalonSRXControlMode.PercentOutput, motorPercentage);
    }

    public double getIntakeMotorCurrent() {
        return m_PDH.getCurrent(9);
    }

    public boolean isHoldingPiece() {
        return getIntakeMotorCurrent() < kHoldingPieceCurrent;
    }

    public double getExtendMotorCurrent() {
        return m_PDH.getCurrent(8);
    }

    public boolean isMaxExtendIn() {
        return getExtendMotorCurrent() > kMaxExtendInCurrentLimit;
    }

    @Override
    public void periodic() {
        m_motorPercentageEntry.getEntry().setDouble(getIntakeMotorCurrent());
        m_isLimitTriggeredEntry.getEntry().setBoolean(isLimitTriggered());
    }

}
