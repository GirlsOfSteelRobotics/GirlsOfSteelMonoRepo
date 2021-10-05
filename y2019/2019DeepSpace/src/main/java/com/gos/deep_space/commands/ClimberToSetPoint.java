/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.gos.deep_space.commands;

import com.gos.deep_space.Robot;
import com.gos.deep_space.subsystems.Climber.ClimberType;
import edu.wpi.first.wpilibj.command.Command;

public class ClimberToSetPoint extends Command {

    private final double setPoint;
    private final ClimberType type;

    public ClimberToSetPoint(double setPoint, ClimberType climberType) {
        this.setPoint = setPoint;
        type = climberType;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.climber);
        requires(Robot.blinkin);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.climber.setGoalClimberPosition(setPoint, type);
        System.out.println("init Climber To " + setPoint);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        System.out.println("goal front position: " + Robot.climber.goalFrontPosition + "actual front position: "
            + Robot.climber.getFrontPosition());
        System.out.println("goal back position" + Robot.climber.goalBackPosition + "actual back position: "
            + Robot.climber.getBackPosition());

        Robot.climber.holdClimberPosition(type);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.climber.checkCurrentPosition(setPoint, type);

    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.climber.climberStop();
        System.out.println("end Climber To " + setPoint);

    }

}
