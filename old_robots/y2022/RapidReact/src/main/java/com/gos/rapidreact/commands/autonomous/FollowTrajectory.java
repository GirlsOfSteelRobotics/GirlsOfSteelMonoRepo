package com.gos.rapidreact.commands.autonomous;

import com.gos.rapidreact.subsystems.ChassisSubsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import org.snobotv2.coordinate_gui.commands.BaseRamseteCoordinateGuiCommand;

public class FollowTrajectory extends BaseRamseteCoordinateGuiCommand {

    private final ChassisSubsystem m_chassis;
    private final Trajectory m_trajectory;

    public static class AutoConstants {
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }


    @SuppressWarnings("removal")
    public FollowTrajectory(Trajectory trajectory, ChassisSubsystem chassis) {
        super(trajectory,
            new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
            ChassisSubsystem.K_DRIVE_KINEMATICS,
            chassis);

        this.m_chassis = chassis;
        this.m_trajectory = trajectory;
    }

    public Pose2d getInitialPose() {
        return m_trajectory.getInitialPose();
    }

    @Override
    protected Pose2d getPose() {
        return m_chassis.getPose();
    }

    @Override
    protected void setVelocity(double leftVelocityMps, double rightVelocityMps, double leftAccelMpss, double rightAccelMpss) {
        m_chassis.smartVelocityControl(m_goalVelocityLeft, m_goalVelocityRight, leftAccelMpss, rightAccelMpss);
    }

    @Override
    protected DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_chassis.getLeftEncoderSpeed(), m_chassis.getRightEncoderSpeed());
    }

    @Override
    public String getName() {
        return "FollowTrajectory";
    }
}
