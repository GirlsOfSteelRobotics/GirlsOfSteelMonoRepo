/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3504.robot;

import org.usfirst.frc.team3504.robot.commands.Collect;
import org.usfirst.frc.team3504.robot.commands.DriveByDistance;
import org.usfirst.frc.team3504.robot.commands.DriveByMotionProfile;
import org.usfirst.frc.team3504.robot.commands.LiftDown;
import org.usfirst.frc.team3504.robot.commands.LiftToSwitch;
import org.usfirst.frc.team3504.robot.commands.LiftUp;
import org.usfirst.frc.team3504.robot.commands.Release;
import org.usfirst.frc.team3504.robot.commands.ShiftDown;
import org.usfirst.frc.team3504.robot.commands.ShiftUp;
import org.usfirst.frc.team3504.robot.commands.WristHold;
import org.usfirst.frc.team3504.robot.commands.WristIn;
import org.usfirst.frc.team3504.robot.commands.WristOut;
import org.usfirst.frc.team3504.robot.subsystems.Shifters;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public enum DriveStyle {
		joystickArcade, gamePadArcade, joystickTank, gamePadTank, amazonDrive
	}; 
	
	private DriveStyle driveStyle;
	
	private Joystick operatorGamePad = new Joystick (0);
	private Joystick drivingGamePad = new Joystick (1);
	private Joystick amazonGamePad = new Joystick (1);
	private Joystick drivingJoystickOne = new Joystick (1);
	private Joystick drivingJoystickTwo = new Joystick (2);
	
	private JoystickButton shifterUp;
	private JoystickButton shifterDown;
	private JoystickButton driveByDistanceLow;
	private JoystickButton driveByMotionProfile;
	
	private JoystickButton liftUp;
	private JoystickButton liftDown;
	private JoystickButton liftToSwitch;
	
	private JoystickButton wristIn;
	private JoystickButton wristOut;
	private JoystickButton wristHold;
	
	private JoystickButton collect;
	private JoystickButton release;
	
	public OI() {
		shifterDown = new JoystickButton(drivingJoystickOne, 3);
		shifterUp = new JoystickButton(drivingJoystickOne, 2);
		driveByDistanceLow = new JoystickButton(drivingJoystickOne, 9);
		driveByMotionProfile = new JoystickButton(drivingJoystickOne, 10);
		
		liftUp = new JoystickButton(operatorGamePad, 10); //TODO: random buttom assignment
		liftDown = new JoystickButton(operatorGamePad, 9);
		liftToSwitch = new JoystickButton(operatorGamePad, 8);
		
		wristIn = new JoystickButton(operatorGamePad, 5);  
		wristOut = new JoystickButton(operatorGamePad, 7);
		wristHold = new JoystickButton(operatorGamePad, 1);
		
		collect = new JoystickButton(operatorGamePad, 3);
		release = new JoystickButton(operatorGamePad, 2);
		
		shifterDown.whenPressed(new ShiftDown());
		shifterUp.whenPressed(new ShiftUp());
		driveByDistanceLow.whenPressed(new DriveByDistance(-12.0, Shifters.Speed.kLow));
		
		//turn left:
		driveByMotionProfile.whenPressed(new DriveByMotionProfile("/home/lvuser/shortTurn2018.dat", "/home/lvuser/longTurn2018.dat"));
		
		liftUp.whileHeld(new LiftUp());
		liftDown.whileHeld(new LiftDown());
		liftToSwitch.whenPressed(new LiftToSwitch());
		
		wristIn.whileHeld(new WristIn());
		wristOut.whileHeld(new WristOut());
		wristHold.whenPressed(new WristHold());
		
		collect.whileHeld(new Collect());
		release.whileHeld(new Release());
		
		drivingGamePad.setTwistChannel(3);
	}
	
	public double getGamePadLeftUpAndDown() {
		return -drivingGamePad.getY();
	}
	
	public double getGamePadRightUpAndDown() {
		return -drivingGamePad.getTwist();
	}
	
	public double getGamePadLeftSideToSide() {
		return drivingGamePad.getX();
	}
	
	public double getGamePadRightSideToSide(){
		return drivingGamePad.getZ();
	}
	
	public double getAmazonLeftUpAndDown() {
		return -amazonGamePad.getY();
	}
	
	public double getAmazonRightSideToSide() {
		return amazonGamePad.getZ();
	}
	
	public double getJoystickOneUpAndDown() {
		return -drivingJoystickOne.getY();
	}
	
	public double getJoystickOneSideToSide() {
		return drivingJoystickOne.getX();
	}
	
	public double getJoystickTwoUpAndDown() {
		return -drivingJoystickTwo.getY();
	}
	
	public double getJoystickTwoSideToSide() {
		return drivingJoystickTwo.getX();
	}
	
	public void setDriveStyle() {
		if (!RobotMap.dio1.get()) {
			driveStyle = DriveStyle.joystickArcade; 
		} else if (!RobotMap.dio2.get()) {
			driveStyle = DriveStyle.gamePadArcade; 
		} else if (!RobotMap.dio3.get()) {
			driveStyle = DriveStyle.joystickTank; 
		} else if (!RobotMap.dio4.get()) {
			driveStyle = DriveStyle.gamePadTank; 
		} else if (!RobotMap.dio5.get()) {
			driveStyle = DriveStyle.amazonDrive;
		} else {
			System.out.println("NO DRIVE MODE SELECTED. \nDefaulting to Joystick Arcade...");
			driveStyle = DriveStyle.joystickArcade; 
		}
		System.out.println("Drive Mode: " + driveStyle);
	}
	
	public DriveStyle getDriveStyle() {
		return driveStyle; 
	}
	
	public boolean isSquaredOrCurvature() { //TODO: rethink boolean
		return !RobotMap.dio0.get(); 
	}
}

