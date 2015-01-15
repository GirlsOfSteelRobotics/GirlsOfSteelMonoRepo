package org.usfirst.frc.team3504.robot.commands;

import org.usfirst.frc.team3504.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/*
 * 
 */
public class AngleSuckerOut extends Command{

	public AngleSuckerOut() {
		requires(Robot.sucker);
	}
	
	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		Robot.sucker.suckerAngleOut();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
	}

	@Override
	protected void interrupted() {
		end();	
	}

}
