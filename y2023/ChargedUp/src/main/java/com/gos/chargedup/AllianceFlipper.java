package com.gos.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.frc2023.FieldConstants;

public class AllianceFlipper {

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
            flipX(translation.getX()),
            translation.getY()
        );
    }

    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.plus(Rotation2d.fromDegrees(180));
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(
            flip(pose.getTranslation()),
            flip(pose.getRotation())
        );
    }

    public static double flipX(double xPos) {
        return FieldConstants.FIELD_LENGTH - xPos;
    }

    public static Translation2d maybeFlip(Translation2d translation) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return flip(translation);
        }
        return translation;
    }

    public static Pose2d maybeFlip(Pose2d pose) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return flip(pose);
        }
        return pose;
    }
}