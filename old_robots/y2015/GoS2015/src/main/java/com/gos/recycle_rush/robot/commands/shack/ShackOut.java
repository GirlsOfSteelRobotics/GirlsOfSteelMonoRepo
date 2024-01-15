package com.gos.recycle_rush.robot.commands.shack;

import edu.wpi.first.wpilibj2.command.Command;
import com.gos.recycle_rush.robot.subsystems.Shack;

/**
 *
 */
public class ShackOut extends Command {

    private final Shack m_shack;

    public ShackOut(Shack shack) {
        m_shack = shack;
        addRequirements(m_shack);
    }

    @Override
    public void initialize() {
        m_shack.shackOut();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }


}
