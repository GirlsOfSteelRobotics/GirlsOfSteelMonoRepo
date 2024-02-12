package com.gos.crescendo2024.commands;

import com.gos.crescendo2024.subsystems.ArmPivotSubsystem;
import com.gos.crescendo2024.subsystems.ChassisSubsystem;
import com.gos.crescendo2024.subsystems.IntakeSubsystem;
import com.gos.crescendo2024.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CombinedCommands {


    public static Command intakePieceCommand(ArmPivotSubsystem armPivot, IntakeSubsystem intake) {
        return armPivot.createMoveArmToGroundIntakeAngleCommand()
            .alongWith(intake.createMoveIntakeInCommand())
            .until(intake::hasGamePiece)
            .withName("Intake Piece");
    }


    public static Command speakerAimAndShoot(ArmPivotSubsystem armPivot, ShooterSubsystem shooter, ChassisSubsystem chassis, IntakeSubsystem intake) {
        return SpeakerAimAndShootCommand.createWithDefaults(armPivot, chassis, intake, shooter)
            .withName("Auto shoot into speaker");
    }

    public static Command speakerAimAndShoot(ArmPivotSubsystem armPivot, ShooterSubsystem shooter, ChassisSubsystem chassis, IntakeSubsystem intake, Supplier<Pose2d> poseSupplier, DoubleSupplier shooterRpmGoalSupplier, DoubleSupplier armAngleGoalSupplier) {
        return new SpeakerAimAndShootCommand(armPivot, chassis, intake, shooter, poseSupplier, shooterRpmGoalSupplier, armAngleGoalSupplier)
            .withName("Auto shoot into speaker");
    }
    public static Command speakerAimAndShoot(ArmPivotSubsystem armPivot, ShooterSubsystem shooter, ChassisSubsystem chassis, IntakeSubsystem intake, Supplier<Pose2d> poseSupplier) {
        return new SpeakerAimAndShootCommand(armPivot, chassis, intake, shooter, poseSupplier)
            .withName("Auto shoot into speaker");
    }

    public static Command prepareSpeakerShot(ArmPivotSubsystem armPivot, ShooterSubsystem shooter) {
        return armPivot.createMoveArmToDefaultSpeakerAngleCommand()
            .alongWith(shooter.createSetRPMCommand(4000));
    }

    public static Command prepareAmpShot(ArmPivotSubsystem armPivot, ShooterSubsystem shooter) {
        return armPivot.createMoveArmToAmpAngleCommand()
            .alongWith(shooter.createSetRPMCommand(400))
            .withName("Prepare Amp Shot");
    }

    public static Command ampShooterCommand(ArmPivotSubsystem armPivot, ShooterSubsystem shooter, IntakeSubsystem intake) {
        return prepareAmpShot(armPivot, shooter)
            .alongWith(intake.createMoveIntakeInCommand())
            .withName("Auto shoot into amp");
    }
}

