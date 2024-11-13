package com.gos.chargedup.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.gos.chargedup.AllianceFlipper;
import com.gos.chargedup.Constants;
import com.gos.chargedup.GosField;
import com.gos.chargedup.RectangleInterface;
import com.gos.lib.GetAllianceUtil;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.phoenix6.alerts.PigeonAlerts;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.HeavyDoubleProperty;
import com.gos.lib.properties.feedforward.SimpleMotorFeedForwardProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiProfiledPidPropertyBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.FieldConstants;


import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Degrees;

public abstract class BaseChassis extends SubsystemBase implements ChassisSubsystemInterface {
    protected static final double PITCH_LOWER_LIMIT = -3.0;
    protected static final double PITCH_UPPER_LIMIT = 3.0;
    protected final HeavyDoubleProperty m_turnPidAllowableError;

    protected final Pigeon2 m_gyro;

    protected final GosField m_field;

    protected final List<Vision> m_cameras;
    protected final ProfiledPIDController m_turnAnglePID;
    protected final PidProperty m_turnAnglePIDProperties;
    protected final SimpleMotorFeedForwardProperty m_turnAnglePIDFFProperty;

    protected final LoggingUtil m_networkTableEntries;
    protected final NetworkTableEntry m_gyroAngleGoalVelocityEntry;

    protected final NetworkTableEntry m_trajectoryVelocitySetpointX;
    protected final NetworkTableEntry m_trajectoryVelocitySetpointY;
    protected final NetworkTableEntry m_trajectoryVelocitySetpointAngle;
    private final NetworkTableEntry m_trajectoryErrorX;
    private final NetworkTableEntry m_trajectoryErrorY;
    private final NetworkTableEntry m_trajectoryErrorAngle;
    private final RectangleInterface m_communityRectangle1;
    private final RectangleInterface m_communityRectangle2;
    private final RectangleInterface m_loadingRectangle1;
    private final RectangleInterface m_loadingRectangle2;
    protected boolean m_tryingToEngage;
    protected final PigeonAlerts m_pigeonAlerts;

    public BaseChassis() {
        m_field = new GosField();
        SmartDashboard.putData(m_field.getSendable());

        m_communityRectangle1 = new RectangleInterface(FieldConstants.Community.INNER_X, FieldConstants.Community.LEFT_Y, FieldConstants.Community.MID_X, FieldConstants.Community.RIGHT_Y, m_field, "BlueCommunityRight");
        m_communityRectangle2 = new RectangleInterface(FieldConstants.Community.MID_X, FieldConstants.Community.MID_Y, FieldConstants.Community.OUTER_X, FieldConstants.Community.RIGHT_Y, m_field, "BlueCommunityLeft");
        m_loadingRectangle1 = new RectangleInterface(FieldConstants.LoadingZone.OUTER_X, FieldConstants.LoadingZone.LEFT_Y, FieldConstants.LoadingZone.MID_X, FieldConstants.LoadingZone.MID_Y, m_field, "BlueLoadingRight");
        m_loadingRectangle2 = new RectangleInterface(FieldConstants.LoadingZone.MID_X, FieldConstants.LoadingZone.LEFT_Y, FieldConstants.LoadingZone.INNER_X, FieldConstants.LoadingZone.RIGHT_Y, m_field, "BlueLoadingLeft");

        m_turnAnglePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        m_turnAnglePID.enableContinuousInput(0, 360);
        m_turnAnglePIDProperties = new WpiProfiledPidPropertyBuilder("Chassis to angle", false, m_turnAnglePID)
            .addP(0.4)
            .addI(0)
            .addD(0)
            .addMaxAcceleration(210)
            .addMaxVelocity(210)
            .build();
        m_turnAnglePIDFFProperty = new SimpleMotorFeedForwardProperty("Chassis to angle FF Properties", false)
            .addKs(0.8)
            .addKa(0)
            .addKff(0.027);
        GosDoubleProperty turnPidAllowableErrorProperty = new GosDoubleProperty(false, "Chassis Turn PID Allowable Error", 3.5);
        m_turnPidAllowableError = new HeavyDoubleProperty(m_turnAnglePID::setTolerance, turnPidAllowableErrorProperty);

        m_cameras = new ArrayList<>();
        m_cameras.add(new LimelightVisionSubsystem(m_field, "limelight-back"));
        m_cameras.add(new LimelightVisionSubsystem(m_field, "limelight-front"));

        m_networkTableEntries = new LoggingUtil("Chassis Subsystem");

        m_networkTableEntries.addDouble("Position values: X", () -> Units.metersToInches(getPose().getX()));
        m_networkTableEntries.addDouble("Position values: Y", () -> Units.metersToInches(getPose().getY()));
        m_networkTableEntries.addDouble("Position values: theta", () -> getPose().getRotation().getDegrees());
        m_networkTableEntries.addDouble("Chassis Pitch", this::getPitch);
        m_networkTableEntries.addBoolean("In Community", this::isInCommunityZone);
        m_networkTableEntries.addBoolean("In Loading", this::isInLoadingZone);
        m_networkTableEntries.addDouble("pose angle", () -> getPose().getRotation().getDegrees());
        m_networkTableEntries.addBoolean("Field/flip", this::isRedAllianceFlipped);
        m_networkTableEntries.addDouble("Gyro Angle (deg)", this::getYaw);

        m_gyro = new Pigeon2(Constants.PIGEON_PORT);
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        if (Constants.IS_ROBOT_BLOSSOM) {
            m_networkTableEntries.addDouble("Gyro Rate", () -> -m_gyro.getRate());
        } else {
            m_networkTableEntries.addDouble("Gyro Rate", () -> m_gyro.getRate());
        }

        NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Chassis Subsystem");
        m_gyroAngleGoalVelocityEntry = loggingTable.getEntry("Gyro Goal Velocity");

        m_trajectoryErrorX = loggingTable.getEntry("Trajectory Error X");
        m_trajectoryErrorY = loggingTable.getEntry("Trajectory Error Y");
        m_trajectoryErrorAngle = loggingTable.getEntry("Trajectory Error Angle");

        m_trajectoryVelocitySetpointX = loggingTable.getEntry("Trajectory Velocity Setpoint X");
        m_trajectoryVelocitySetpointY = loggingTable.getEntry("Trajectory Velocity Setpoint Y");
        m_trajectoryVelocitySetpointAngle = loggingTable.getEntry("Trajectory Velocity Setpoint Angle");

        m_pigeonAlerts = new PigeonAlerts(m_gyro);


    }

    protected void logTrajectoryErrors(Translation2d translation2d, Rotation2d rotation2d) {
        m_trajectoryErrorX.setNumber(translation2d.getX());
        m_trajectoryErrorY.setNumber(translation2d.getY());
        m_trajectoryErrorAngle.setNumber(rotation2d.getDegrees());
    }

    @Override
    public double findingClosestNodeY(double yPositionButton) {
        double distanceBetweenArrays = FieldConstants.Grids.NODE_SEPARATION_Y * 3;
        double array1 = yPositionButton + 0 * distanceBetweenArrays;
        double array2 = yPositionButton + 1 * distanceBetweenArrays;
        double array3 = yPositionButton + 2 * distanceBetweenArrays;
        double[] mClosestArray = {array1, array2, array3};

        final Pose2d currentRobotPosition = getPose();
        double currentYPos = currentRobotPosition.getY();
        double minDist = Integer.MAX_VALUE;
        double closestNode = minDist;
        for (int i = 0; i < 3; i++) {
            double currentDistance = Math.abs(currentYPos - mClosestArray[i]);
            if (currentDistance < minDist) {
                closestNode = mClosestArray[i];
                minDist = currentDistance;
            }
        }

        return closestNode;
    }

    @Override
    public double getPitch() {
        // INTENTIONALLY ROLL, WE ARE NOT BEING PSYCHOPATHS I PROMISE
        return m_gyro.getRoll().getValue().in(Degrees);
    }

    @Override
    public double getYaw() {
        return m_gyro.getYaw().getValue().in(Degrees);
    }

    @Override
    public boolean tryingToEngage() {
        return m_tryingToEngage;
    }

    @Override
    public boolean turnPIDIsAtAngle() {
        return m_turnAnglePID.atGoal();
    }

    @Override
    public boolean isInCommunityZone() {
        return m_communityRectangle1.pointIsInRect(getPose())
            || m_communityRectangle2.pointIsInRect(getPose());
    }

    @Override
    public boolean isInLoadingZone() {
        return m_loadingRectangle1.pointIsInRect(getPose())
            || m_loadingRectangle2.pointIsInRect(getPose());
    }

    public boolean isRedAllianceFlipped() {
        return GetAllianceUtil.isRedAlliance();
    }

    @Override
    public boolean canExtendArm() {
        return (this.isInCommunityZone() || this.isInLoadingZone());
    }

    protected abstract void lockDriveTrain();

    protected abstract void unlockDriveTrain();

    //////////////////////////////
    // Commands
    //////////////////////////////
    @Override
    @SuppressWarnings("PMD.AvoidReassigningParameters")
    public Command createDriveToPointCommand(Pose2d point, boolean reverse) {
        point = AllianceFlipper.maybeFlip(point);
        System.out.println("flipped point" + point);
        return createDriveToPointNoFlipCommand(getPose(), point, reverse);
    }

    @Override
    public Command createAutoEngageCommand() {
        return this.runOnce(this::lockDriveTrain)
            .andThen(runEnd(this::autoEngage, () -> m_tryingToEngage = false))
            .finallyDo((interrupted) -> unlockDriveTrain()).withName("Auto Engage");
    }

    @Override
    public Command createResetOdometryCommand(Pose2d pose2d) {
        return this.run(() -> resetOdometry(pose2d))
            .ignoringDisable(true)
            .withName("Reset Odometry [" + pose2d.getX() + ", " + pose2d.getY() + ", " + pose2d.getRotation().getDegrees() + "]");
    }

    @Override
    public Command createDeferredDriveToPointCommand(Pose2d point, boolean reverse) {
        return defer(() -> createDriveToPointCommand(point, reverse));
    }

    @Override
    public Command createTurnToAngleCommand(double angleGoal) {
        return runOnce(() -> m_turnAnglePID.reset(getPose().getRotation().getDegrees()))
            .andThen(this.run(() -> turnToAngle(angleGoal))
                .until(this::turnPIDIsAtAngle)
                .withName("Chassis to Angle" + angleGoal));
    }
}
