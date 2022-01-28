package com.gos.rapidreact.commands.tuning;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.rapidreact.subsystems.CollectorSubsystem;


public class TuneCollectorPivotPIDGravityOffsetCommand extends CommandBase {
    private final CollectorSubsystem m_collector;

    public TuneCollectorPivotPIDGravityOffsetCommand(CollectorSubsystem collectorSubsystem) {
        this.m_collector = collectorSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.m_collector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_collector.tuneGravityOffset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_collector.pivotStop();
    }
}
