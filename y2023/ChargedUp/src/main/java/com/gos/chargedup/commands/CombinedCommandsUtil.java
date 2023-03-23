package com.gos.chargedup.commands;

import com.gos.chargedup.AutoPivotHeight;
import com.gos.chargedup.GamePieceType;
import com.gos.chargedup.subsystems.ArmExtensionSubsystem;
import com.gos.chargedup.subsystems.ArmPivotSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public final class CombinedCommandsUtil {

    private CombinedCommandsUtil() {

    }

    public static CommandBase goHome(ArmPivotSubsystem pivot, ArmExtensionSubsystem extension) {
        return extension.commandFullRetract()
            .andThen(pivot.commandGoHome())
            .withName("Go Home Without Turret");
    }

    //public static CommandBase goHomeWithTurret(ArmPivotSubsystem pivot, ArmExtensionSubsystem extension, TurretSubsystem turret) {
    //    return extension.commandFullRetract()
    //        .andThen(pivot.commandGoHome()
    //            .alongWith(turret.goHome()))
    //        .withName("Go Home With Turret");
    //}

    public static CommandBase goToGroundPickup(ArmPivotSubsystem pivot, ArmExtensionSubsystem extension) {
        return extension.commandMiddleRetract()
            .alongWith(pivot.commandGoToGroundPickup())
            .withName("Go To Ground Pickup");
    }

    public static CommandBase armToHpPickup(ArmPivotSubsystem pivot, ArmExtensionSubsystem extension) {
        return pivot.commandHpPickupHold()
            .alongWith(extension.commandMiddleRetract())
            .withName("HP Pickup");
    }

    public static CommandBase moveToScore(AutoPivotHeight height, GamePieceType gamePiece, ArmPivotSubsystem armPivot) {
        return new ParallelCommandGroup(
            //turret.commandTurretPID(turretAngle),
            armPivot.commandMoveArmToPieceScorePositionAndHold(height, gamePiece) //set for second piece
        ).withName("Move To Score");
    }
}
