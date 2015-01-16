package org.usfirst.frc.team3504.robot.subsystems;

import org.usfirst.frc.team3504.robot.RobotMap;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;

public class UltrasonicSensor extends Subsystem{

	private Ultrasonic ultrasonicSensor;
	
	public UltrasonicSensor() {
		ultrasonicSensor = new Ultrasonic(RobotMap.ULTRASONICSENSOR_PING_CHANNEL, RobotMap.ULTRASONICSENSOR_ECHO_CHANNEL);
	}

	public double getDistanceInches() {
		return ultrasonicSensor.getRangeInches();
	}
	
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
