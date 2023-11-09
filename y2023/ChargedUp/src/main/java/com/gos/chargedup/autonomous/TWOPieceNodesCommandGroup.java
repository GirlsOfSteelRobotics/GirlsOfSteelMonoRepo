package com.gos.chargedup.autonomous;

import com.gos.chargedup.AutoPivotHeight;
import com.gos.chargedup.GamePieceType;
import com.gos.chargedup.commands.CombinedCommandsUtil;
import com.gos.chargedup.commands.ScorePieceCommandGroup;
import com.gos.chargedup.subsystems.ArmExtensionSubsystem;
import com.gos.chargedup.subsystems.ArmPivotSubsystem;
import com.gos.chargedup.subsystems.ChassisSubsystemInterface;
import com.gos.chargedup.subsystems.ClawSubsystem;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;
import java.util.Map;

public class TWOPieceNodesCommandGroup extends SequentialCommandGroup {
    public static final PathConstraints FASTER_PATH_CONSTRAINTS = new PathConstraints(Units.inchesToMeters(120), Units.inchesToMeters(120));
    public static final PathConstraints NOT_AS_FAST_PATH_CONSTRAINTS = new PathConstraints(Units.inchesToMeters(70), Units.inchesToMeters(70));

    public TWOPieceNodesCommandGroup(ChassisSubsystemInterface chassis, ArmPivotSubsystem armPivot, ArmExtensionSubsystem armExtension, ClawSubsystem claw, AutoPivotHeight pivotHeightType, String pathStart, String pathMiddle, String pathEnd) {

        Map<String, Command> eventMap = new HashMap<>();
        eventMap.put("GrabPiece", CombinedCommandsUtil.goToGroundPickup(armPivot, armExtension, 10, 200000));

        Command driveToFirstPiece = chassis.createFollowPathCommand(pathStart);
        Command driveToGetSecondPiece = chassis.createFollowPathCommandNoPoseReset(pathMiddle);
        Command driveToScoreSecondPiece = chassis.createFollowPathCommandNoPoseReset(pathEnd);

        //score piece
        addCommands(new ScorePieceCommandGroup(armPivot, armExtension, claw, pivotHeightType, GamePieceType.CONE));

        //first part
        addCommands(driveToFirstPiece
            .alongWith(armPivot.createPivotToAngleAndHoldCommand(-10))
            .alongWith(Commands.waitSeconds(0.25)
                .andThen(armExtension.createMiddleExtensionCommand())));

        //turn 180
        addCommands(chassis.createTurnToAngleCommand(0));

        //drive second part
        addCommands(driveToGetSecondPiece
            .raceWith(claw.createMoveClawIntakeInCommand()));

        //turn 180
        addCommands(chassis.createTurnToAngleCommand(180)
            .alongWith(CombinedCommandsUtil.moveToScore(pivotHeightType, GamePieceType.CUBE, armPivot))
            .raceWith(claw.createHoldPieceCommand()));


        //third part
        addCommands(driveToScoreSecondPiece
            .alongWith(armPivot.createMoveArmToPieceScorePositionAndHoldCommand(pivotHeightType, GamePieceType.CUBE))
            .raceWith(claw.createMoveClawIntakeInCommand()));

        //score piece
        addCommands(new ScorePieceCommandGroup(armPivot, armExtension, claw, pivotHeightType, GamePieceType.CUBE));

    }
}
