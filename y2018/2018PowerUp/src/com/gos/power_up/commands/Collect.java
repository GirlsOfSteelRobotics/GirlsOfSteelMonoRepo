package com.gos.power_up.commands;

import com.gos.power_up.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Collect extends Command {

    public Collect() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.collector);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.collector.collect();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.collector.runSlowCollect();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
