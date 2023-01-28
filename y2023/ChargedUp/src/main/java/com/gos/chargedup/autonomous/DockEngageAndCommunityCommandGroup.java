package com.gos.chargedup.autonomous;

import com.gos.chargedup.subsystems.ChassisSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DockEngageAndCommunityCommandGroup extends SequentialCommandGroup {
    public static final PathPlannerTrajectory DOCK_ENGAGE_AND_COMMUNITY = PathPlanner.loadPath("DockAndEngage,Community", new PathConstraints(4, 3));
    public DockEngageAndCommunityCommandGroup(ChassisSubsystem chassis) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
            chassis.followTrajectoryCommand(DOCK_ENGAGE_AND_COMMUNITY, true)
        );
    }
}