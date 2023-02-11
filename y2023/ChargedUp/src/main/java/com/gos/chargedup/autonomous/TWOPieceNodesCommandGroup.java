package com.gos.chargedup.autonomous;

import com.gos.chargedup.Constants;
import com.gos.chargedup.commands.ScorePieceCommandGroup;
import com.gos.chargedup.subsystems.ArmSubsystem;
import com.gos.chargedup.subsystems.ChassisSubsystem;
import com.gos.chargedup.subsystems.ClawSubsystem;
import com.gos.chargedup.subsystems.IntakeSubsystem;
import com.gos.chargedup.subsystems.TurretSubsystem;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;
import java.util.List;


public class TWOPieceNodesCommandGroup extends SequentialCommandGroup {

    public TWOPieceNodesCommandGroup(ChassisSubsystem chassis, TurretSubsystem turret, ArmSubsystem arm, ClawSubsystem claw, IntakeSubsystem intake, String autoName) {

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("pickUpObject", new SequentialCommandGroup(
            //middle part
            claw.createMoveClawIntakeInCommand() //piece is firmly in the claw? finish + tune soon
        ));

        List<PathPlannerTrajectory> TWOPieceNodes0And1 = PathPlanner.loadPathGroup(autoName, Constants.DEFAULT_PATH_CONSTRAINTS);
        Command fullAuto = chassis.ramseteAutoBuilder(eventMap).fullAuto(TWOPieceNodes0And1);

        //score first piece:
        addCommands(new ScorePieceCommandGroup(turret, arm, claw));

        //drive, get piece, drive back
        addCommands(fullAuto);

        //score piece:
        addCommands(new ScorePieceCommandGroup(turret, arm, claw));

    }
}