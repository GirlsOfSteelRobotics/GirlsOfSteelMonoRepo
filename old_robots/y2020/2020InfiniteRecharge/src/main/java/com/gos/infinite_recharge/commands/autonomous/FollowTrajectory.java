package com.gos.infinite_recharge.commands.autonomous;

import com.gos.infinite_recharge.Constants;
import com.gos.infinite_recharge.subsystems.Chassis;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import org.snobotv2.coordinate_gui.commands.BaseRamseteCoordinateGuiCommand;

public class FollowTrajectory extends BaseRamseteCoordinateGuiCommand {

    private final Chassis m_chassis;

    public static class AutoConstants {
        public static final double SLOW_SPEED_METERS_PER_SECOND = Units.inchesToMeters(48);
        public static final double SLOW_ACCELERATION_METERS_PER_SECOND_SQUARED = Units.inchesToMeters(96);
        public static final double NORMAL_SPEED_METERS_PER_SECOND = Units.inchesToMeters(72);
        public static final double NORMAL_ACCELERATION_METERS_PER_SECOND_SQUARED = Units.inchesToMeters(60);
        public static final double FAST_SPEED_METERS_PER_SECOND = Units.inchesToMeters(120);
        public static final double FAST_ACCELERATION_METERS_PER_SECOND_SQUARED = Units.inchesToMeters(120);

        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

    }


    @SuppressWarnings("removal")
    public FollowTrajectory(Trajectory trajectory, Chassis chassis) {
        super(trajectory,
            new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
            Constants.DriveConstants.DRIVE_KINEMATICS,
            chassis);

        this.m_chassis = chassis;
    }

    @Override
    protected Pose2d getPose() {
        return m_chassis.getPose();
    }

    @Override
    protected void setVelocity(double leftVelocityMps, double rightVelocityMps, double leftAccelMpss, double rightAccelMpss) {
        double leftVelocityInchesPerSecond = Units.metersToInches(leftVelocityMps);
        double rightVelocityInchesPerSecond = Units.metersToInches(rightVelocityMps);
        m_goalVelocityLeft = leftVelocityInchesPerSecond;
        m_goalVelocityRight = rightVelocityInchesPerSecond;
        //System.out.println("Setting goal velocity: " + m_goalVelocityLeft + ", " + m_goalVelocityRight);
        m_chassis.smartVelocityControl(m_goalVelocityLeft, m_goalVelocityRight);
    }

    @Override
    protected DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(m_chassis.getLeftEncoderSpeed()), Units.inchesToMeters(m_chassis.getRightEncoderSpeed()));
    }

    @Override
    public String getName() {
        return "FollowTrajectory";
    }
}
