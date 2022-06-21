package com.gos.stronghold.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.gos.stronghold.robot.RobotMap;
import com.gos.stronghold.robot.subsystems.Claw;
import com.gos.stronghold.robot.subsystems.Shooter;

/**
 *
 */
public class StopShooterWheels extends CommandBase {

    private final Claw m_claw;
    private final Shooter m_shooter;

    public StopShooterWheels(Claw claw, Shooter shooter) {
        m_claw = claw;
        m_shooter = shooter;

        if (RobotMap.USING_CLAW) {
            addRequirements(m_claw);
        } else {
            addRequirements(m_shooter);
        }
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (RobotMap.USING_CLAW) {
            m_claw.stopCollecting();
        } else {
            m_shooter.stop();
        }
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
