package com.gos.chargedup.autonomous;


import com.gos.chargedup.AutoEnumsWithScorePiecePivot;
import com.gos.chargedup.GamePieceType;
import com.gos.chargedup.commands.ScorePieceCommandGroup;
import com.gos.chargedup.subsystems.ArmSubsystem;
import com.gos.chargedup.subsystems.ClawSubsystem;
import com.gos.chargedup.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreHighConeAtCurrentPosCommandGroup extends SequentialCommandGroup {
    public ScoreHighConeAtCurrentPosCommandGroup(TurretSubsystem turret, ArmSubsystem arm, ClawSubsystem claw, AutoEnumsWithScorePiecePivot piece, GamePieceType gamePieceType) {
        //addCommands(turret.commandTurretPID(180));
        addCommands(new ScorePieceCommandGroup(turret, arm, claw, piece, gamePieceType));
    }
}
