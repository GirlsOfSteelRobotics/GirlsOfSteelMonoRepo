package com.gos.power_up.commands;

import com.gos.power_up.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;

/**
 *
 */
public class WristIn extends Command {
    private final Wrist m_wrist;

    public WristIn(Wrist wrist) {
        m_wrist = wrist;
        addRequirements(m_wrist);
    }


    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        m_wrist.holdWristPosition();
        m_wrist.wristIn();
    }


    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {

    }


}
