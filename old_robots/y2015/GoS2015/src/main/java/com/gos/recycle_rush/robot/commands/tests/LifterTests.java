package com.gos.recycle_rush.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.recycle_rush.robot.subsystems.Lifter;

/**
 *
 */
public class LifterTests extends CommandBase {

    private final Lifter m_lifter;

    public LifterTests(Lifter lifter) {
        m_lifter = lifter;
        addRequirements(m_lifter);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_lifter.printLifter();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }


}
