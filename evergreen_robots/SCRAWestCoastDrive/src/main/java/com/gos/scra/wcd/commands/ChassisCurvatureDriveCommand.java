package com.gos.scra.wcd.commands;

import com.gos.scra.wcd.subsystems.ChassisSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class ChassisCurvatureDriveCommand extends Command {
    private final ChassisSubsystem m_chassis;
    private final XboxController m_joystick;

    public ChassisCurvatureDriveCommand(ChassisSubsystem chassisSubsystem, XboxController joystick) {
        m_chassis = chassisSubsystem;
        m_joystick = joystick;
        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_chassis.setCurvatureDrive(-m_joystick.getLeftY(), -m_joystick.getRightX(), m_joystick.getRightBumperButton());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
