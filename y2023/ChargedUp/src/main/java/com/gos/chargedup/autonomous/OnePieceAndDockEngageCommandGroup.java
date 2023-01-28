package com.gos.chargedup.autonomous;

import com.gos.chargedup.subsystems.ChassisSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceAndDockEngageCommandGroup extends SequentialCommandGroup {
    public static final PathPlannerTrajectory ONE_PIECE_AND_DOCK_ENGAGE = PathPlanner.loadPath("OnePiece,DockAndEngage", new PathConstraints(4, 3));
    public OnePieceAndDockEngageCommandGroup(ChassisSubsystem chassis) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
            chassis.followTrajectoryCommand(ONE_PIECE_AND_DOCK_ENGAGE, true)
        );
    }
}