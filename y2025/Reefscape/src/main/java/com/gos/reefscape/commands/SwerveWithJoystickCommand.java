package com.gos.reefscape.commands;

import com.gos.reefscape.subsystems.drive.GOSSwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class SwerveWithJoystickCommand extends Command {
    private final GOSSwerveDrive m_chassis;
    private final CommandXboxController m_joystick;

    public SwerveWithJoystickCommand(GOSSwerveDrive chassisSubsystem, CommandXboxController joystick) {
        this.m_chassis = chassisSubsystem;
        m_joystick = joystick;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.m_chassis);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_chassis.driveWithJoystick(
            -m_joystick.getLeftY(),
            m_joystick.getLeftX(),
            m_joystick.getRightX());

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.driveWithJoystick(0, 0, 0);
    }
}
