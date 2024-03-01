package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final ElevatorSubsystem m_subsystem;
    private double m_motorVoltage;

    public ElevatorDownCommand(ElevatorSubsystem subsystem) {
        this.m_motorVoltage = -4;
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setElevatorMotorsVoltage(m_motorVoltage);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setElevatorMotorsVoltage(0.25);
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isUpperLimitTriggered();
    }

}