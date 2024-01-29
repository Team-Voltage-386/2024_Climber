package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.components.ElevatorDirection;

public class MotorTest extends SubsystemBase 
{
    private DigitalInput HighSwitch;
    private DigitalInput LowSwitch;

    private static final String MOTOR_A_NAME = "A";
    private static final String MOTOR_B_NAME = "B";

    private ShuffleboardTab liftTab;

    private CANSparkMax m_motorA;
    private CANSparkMax m_motorB;
    private Command m_motorACurrentCommand;
    private Command m_motorBCurrentCommand;

    private SimpleWidget topLimit;
    private SimpleWidget lowLimit;
    private SimpleWidget motorVolt;

    public MotorTest() 
    {
        liftTab = Shuffleboard.getTab("Lift Controls");

        HighSwitch = new DigitalInput(3);
        LowSwitch = new DigitalInput(4);

        m_motorA = new CANSparkMax(MotorConstants.kDeviceIDMotorA, MotorType.kBrushless);
        //m_motorB = new CANSparkMax(MotorConstants.kDeviceIDMotorA, MotorType.kBrushless);
        m_motorA.setVoltage(0);

        topLimit = liftTab.add("Top Limit", false);
        lowLimit = liftTab.add("Bottom Limit", false);
        motorVolt = liftTab.add("Motor Voltage", 0.0);
    }

    @Override
    public void periodic() {
        boolean top = HighSwitch.get();
        topLimit.getEntry().setBoolean(HighSwitch.get()); //3
        boolean low = LowSwitch.get();
        lowLimit.getEntry().setBoolean(LowSwitch.get()); //4

        double volts = motorVolt.getEntry().getDouble(0);
        if (top==true && volts>0)
        {
            m_motorA.setVoltage(volts);
        }
        else 
        {
            if(low==true && volts<0)
            {
                m_motorA.setVoltage(volts);
            }
        }
        if ((low==false && volts<0) || (top==false && volts>0) || volts==0)     
        {
            m_motorA.setVoltage(0);
        }
    }
}