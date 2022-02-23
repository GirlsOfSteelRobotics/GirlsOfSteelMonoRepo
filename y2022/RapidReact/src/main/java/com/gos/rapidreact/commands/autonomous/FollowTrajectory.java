package com.gos.rapidreact.commands.autonomous;

import com.gos.rapidreact.subsystems.ChassisSubsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.snobotv2.coordinate_gui.RamsetePublisher;

public class FollowTrajectory extends SequentialCommandGroup {

    private final ChassisSubsystem m_chassis;
    private final Trajectory m_trajectory;
    private final RamsetePublisher m_ramsetePublisher;

    private double m_goalVelocityLeft;
    private double m_goalVelocityRight;

    public static class AutoConstants {
        public static final double slowSpeedMetersPerSecond = Units.inchesToMeters(48);
        public static final double slowAccelerationMetersPerSecondSquared = Units.inchesToMeters(96);
        public static final double normalSpeedMetersPerSecond = Units.inchesToMeters(72);
        public static final double normalAccelerationMetersPerSecondSquared = Units.inchesToMeters(60);
        public static final double fastSpeedMetersPerSecond = Units.inchesToMeters(120);
        public static final double fastAccelerationMetersPerSecondSquared = Units.inchesToMeters(120);

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public static class DriveConstants {
        public static final double ksVolts = 0.179;
        public static final double kvVoltSecondsPerMeter = 0.0653;
        public static final double kaVoltSecondsSquaredPerMeter = 0.00754;
        public static final double kvVoltSecondsPerRadian = 2.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;
        public static final double maxVoltage = 10;

        public static final double kTrackwidthMeters = 1.1554881713809029;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    }


    public FollowTrajectory(Trajectory trajectory, ChassisSubsystem chassis) {

        this.m_chassis = chassis;
        this.m_trajectory = trajectory;

        m_ramsetePublisher = new RamsetePublisher();

        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            m_chassis::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            DriveConstants.kDriveKinematics,
            this::setVelocityGoal,
            m_chassis
        );

        addCommands(ramseteCommand);
        addCommands(new InstantCommand(() -> m_chassis.stop(), m_chassis));
    }

    private void setVelocityGoal(double leftVelocityMeters, double rightVelocityMeters) {
        m_goalVelocityLeft = leftVelocityMeters;
        m_goalVelocityRight = rightVelocityMeters;
        //System.out.println("Setting goal velocity: " + m_goalVelocityLeft + ", " + m_goalVelocityRight);
        m_chassis.smartVelocityControl(m_goalVelocityLeft, m_goalVelocityRight);
    }

    @Override
    public void initialize() {
        super.initialize();
        m_ramsetePublisher.initialize(m_trajectory);
    }

    @Override
    public void execute() {
        super.execute();

        m_ramsetePublisher.addMeasurement(m_chassis.getPose(),
            new DifferentialDriveWheelSpeeds(Units.inchesToMeters(m_goalVelocityLeft), Units.inchesToMeters(m_goalVelocityRight)),
            new DifferentialDriveWheelSpeeds(Units.inchesToMeters(m_chassis.getLeftEncoderSpeed()), Units.inchesToMeters(m_chassis.getRightEncoderSpeed())));
    }

    @Override
    public String getName() {
        return "FollowTrajectory";
    }
}
