/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimberBackUp extends Command {
  public ClimberBackUp() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Back Up Init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.climber.holdClimberBackPosition();
    Robot.climber.incrementClimber();
    System.out.println("Back Position: " + Robot.climber.getBackPosition());
    System.out.println("Goal Position: "+ Robot.climber.getGoalClimberPosition());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean isFinished = (Robot.climber.getGoalClimberPosition() <= Robot.climber.getBackPosition()+ 500 
    && Robot.climber.getGoalClimberPosition() >= Robot.climber.getBackPosition()-500);
    System.out.println("isFinished: " + isFinished);
    return (Robot.climber.getGoalClimberPosition() <= Robot.climber.getBackPosition()+ 500 
      && Robot.climber.getGoalClimberPosition() >= Robot.climber.getBackPosition()-500);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
