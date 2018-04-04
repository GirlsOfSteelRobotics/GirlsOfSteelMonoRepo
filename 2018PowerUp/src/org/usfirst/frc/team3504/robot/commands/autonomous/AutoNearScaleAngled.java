
package org.usfirst.frc.team3504.robot.commands.autonomous;

import org.usfirst.frc.team3504.robot.GameData.FieldSide;
import org.usfirst.frc.team3504.robot.commands.CollectPosition;
import org.usfirst.frc.team3504.robot.commands.CollectorStop;
import org.usfirst.frc.team3504.robot.commands.DriveByMotionMagic;
import org.usfirst.frc.team3504.robot.commands.LiftHold;
import org.usfirst.frc.team3504.robot.commands.ReleaseFast;
import org.usfirst.frc.team3504.robot.commands.TimeDelay;
import org.usfirst.frc.team3504.robot.commands.WristHold;
import org.usfirst.frc.team3504.robot.commands.WristToCollect;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoNearScaleAngled extends CommandGroup {

	
	//distance constants
	private final double DISTANCE_FORWARD_1 = 225.0;
	private final double DISTANCE_TURN_1 = 25;
	private final double BACK_UP = -30.0;


	public AutoNearScaleAngled(FieldSide robotPosition) {
		System.out.println("AutoFarScale starting");

		//moves robot forward
		addParallel(new DriveByMotionMagic(DISTANCE_FORWARD_1, 0));
		addSequential(new TimeDelay(3.0));
		
		//gets lift & wrist into position
		addSequential(new WristToCollect());
		//addSequential(new LiftToScale());
		addParallel(new WristHold());
		addParallel(new LiftHold());
		addSequential(new TimeDelay(2.0)); 
		
		//turn
		if (robotPosition == FieldSide.left)
			addSequential(new DriveByMotionMagic(DISTANCE_TURN_1, -25));
		else
			addSequential(new DriveByMotionMagic(DISTANCE_TURN_1, 25)); 

		//release cube and back up
		addParallel(new ReleaseFast());
		addSequential(new TimeDelay(1.0));
		addSequential(new DriveByMotionMagic(BACK_UP, 0));

		//puts lift down and stops collector
		addSequential(new CollectPosition());
		addSequential(new CollectorStop());
		addParallel(new WristHold());
		addParallel(new LiftHold());
	}
}

