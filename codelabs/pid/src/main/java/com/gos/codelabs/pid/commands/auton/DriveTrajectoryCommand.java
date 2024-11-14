package com.gos.codelabs.pid.commands.auton;

import com.gos.codelabs.pid.Constants;
import com.gos.codelabs.pid.subsystems.ChassisSubsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import org.snobotv2.coordinate_gui.commands.BaseRamseteCoordinateGuiCommand;

import java.util.function.Supplier;

public class DriveTrajectoryCommand extends BaseRamseteCoordinateGuiCommand {

    private final ChassisSubsystem m_chassis;

    public static Command createWithVelocity(ChassisSubsystem drivetrain, Supplier<Trajectory> trajectorySupplier, boolean resetPosition) {

        DriveTrajectoryCommand runThenStop = new DriveTrajectoryCommand(trajectorySupplier, drivetrain);
        if (resetPosition) {
            Command resetPositionCommand = new SetRobotPoseCommand(drivetrain, runThenStop.getInitialPose());
            return resetPositionCommand.andThen(runThenStop); // NOPMD
        }

        return runThenStop;
    }

    public static class AutoConstants {
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }


    @SuppressWarnings("removal")
    public DriveTrajectoryCommand(Supplier<Trajectory> trajectorySupplier, ChassisSubsystem chassis) {
        super(trajectorySupplier.get(),
                new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
                Constants.DrivetrainConstants.DRIVE_KINEMATICS,
                chassis);

        this.m_chassis = chassis;
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
        m_chassis.driveWithVelocity(m_goalVelocityLeft, m_goalVelocityRight, leftAccelMpss, rightAccelMpss);
    }

    @Override
    protected DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_chassis.getLeftVelocity(), m_chassis.getRightVelocity());
    }

    @Override
    public String getName() {
        return "FollowTrajectory";
    }
}
