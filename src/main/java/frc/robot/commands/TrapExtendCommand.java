package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapSubsystem;

public class TrapExtendCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private double m_motorPercentage;
    private final TrapSubsystem m_subsystem;

    public TrapExtendCommand(double motorPercentage, TrapSubsystem subsystem) {
        this.m_motorPercentage = motorPercentage;
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_subsystem.setTrapExtendMotor(this.m_motorPercentage);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setTrapExtendMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
        // return m_subsystem.isLimitTriggered();
    }

}
