package com.gos.stronghold.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.stronghold.robot.subsystems.Chassis;

/**
 *
 */
public class AutoDoNothing extends CommandBase {

    public AutoDoNothing(Chassis chassis) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        addRequirements(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }


}
