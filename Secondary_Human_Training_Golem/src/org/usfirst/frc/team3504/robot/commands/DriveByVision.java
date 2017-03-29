package org.usfirst.frc.team3504.robot.commands;

import org.usfirst.frc.team3504.robot.Robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveByVision extends Command {

	NetworkTable table;

	private static final double MAX_ANGULAR_VELOCITY = 1.0; //TODO: adjust (rad/s)  current value works
	private static final int IMAGE_WIDTH = 320;
	private static final double IMAGE_CENTER = IMAGE_WIDTH/2.0; 
	double[] defaultValue = new double[0];
	public CANTalon leftTalon = Robot.chassis.getLeftTalon();
	public CANTalon rightTalon = Robot.chassis.getRightTalon();
	private final int TIMEOUT = 8;
	private final int SLIPPING_VELOCITY = 40;
	private Timer tim; 
	private double slowLinearVelocity = 20; //TODO: change (in/s)
	private double fastLinearVelocity = 20; //TODO: change (in/s)
	
	//width of X or Y in pixels when the robot is at the lift
	//private static final double GOAL_WIDTH = 30; //TODO: test and change

	//distance b/w wheels
	private static final double WHEEL_BASE = 24.25; //(in)

	//radius of wheel
	private static final int WHEEL_RADIUS = 2; //(in)

	public DriveByVision() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.chassis);
		tim = new Timer();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
		//not calling setupFPID because other PID values override
		leftTalon.setPosition(0);
		rightTalon.setPosition(0);
    	
		//Change motor control to speed in the -1..+1 range
		leftTalon.changeControlMode(TalonControlMode.Speed);
		rightTalon.changeControlMode(TalonControlMode.Speed);
		
		//tuned by janet and ziya on 2/20, overrides PID set in chassis method
		leftTalon.setF(0.22);  //carpet on practice field
		leftTalon.setP(0.235);
		rightTalon.setF(0.2);
		rightTalon.setP(0.235);
		
		System.out.println("DriveByVision Initialized");
		
		tim.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
		table = NetworkTable.getTable("GRIP/myContoursReport");
		
		double[] centerX = new double[2];
		centerX = table.getNumberArray("centerX", defaultValue);
		/*double[] centerY = new double[2];
		centerY = table.getNumberArray("centerY", defaultValue);*/
		double[] height = new double[2];
		height = table.getNumberArray("height", defaultValue);
		/*double[] width = new double[2];
		width = table.getNumberArray("width", defaultValue);*/

		// the center of the x and y rectangles (the target)
		double goalAngularVelocity;
		if (centerX.length != 2) {
			goalAngularVelocity = 0;
			SmartDashboard.putBoolean("Gear In Sight", false);
		} else {
			double targetX = (centerX[0] + centerX[1])/2.0;
			double error = (targetX - IMAGE_CENTER)/IMAGE_CENTER;
			goalAngularVelocity = error * MAX_ANGULAR_VELOCITY;
			SmartDashboard.putBoolean("Gear In Sight", true);
			SmartDashboard.putNumber("CenterX0", centerX[0]);
			SmartDashboard.putNumber("CenterX1", centerX[1]);
		}
		
		
		double goalLinearVelocity;
		if ((height[0] + height[1]) / 2 >= 52){
			goalLinearVelocity = fastLinearVelocity;
		}
		else{
			goalLinearVelocity = slowLinearVelocity;
		}

		//right and left desired wheel speeds in inches per second
		double vRight = goalLinearVelocity - (WHEEL_BASE * goalAngularVelocity) / 2; //(in/s)
		double vLeft = goalLinearVelocity + (WHEEL_BASE * goalAngularVelocity) / 2;
		//right and left desired wheel speeds in RPM
		double angVRight = 75 * vRight / (2 * Math.PI * WHEEL_RADIUS); //(RPM)
		double angVLeft = 75 * vLeft / (2 * Math.PI * WHEEL_RADIUS);
		//send desired wheel speeds to Talon set to velocity control mode
		rightTalon.set(angVRight);
		leftTalon.set(-angVLeft);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		SmartDashboard.putNumber("Vision Timer", tim.get());
		
		return (tim.get() > 1 && Math.abs(leftTalon.getEncVelocity()) < SLIPPING_VELOCITY && 
				Math.abs(rightTalon.getEncVelocity()) < SLIPPING_VELOCITY) ||
				((tim.get() > TIMEOUT));
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("DriveByVision Finished");
		tim.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
