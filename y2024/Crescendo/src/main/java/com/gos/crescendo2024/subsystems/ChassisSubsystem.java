// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gos.crescendo2024.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.gos.crescendo2024.Constants;
import com.gos.crescendo2024.FieldConstants;
import com.gos.crescendo2024.GoSField;
import com.gos.crescendo2024.MaybeFlippedPose2d;
import com.gos.crescendo2024.ObjectDetection;
import com.gos.crescendo2024.RobotExtrinsics;
import com.gos.crescendo2024.ValidShootingPolygon;
import com.gos.lib.GetAllianceUtil;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.photonvision.AprilTagCamera;
import com.gos.lib.photonvision.AprilTagCameraManager;
import com.gos.lib.properties.GosBooleanProperty;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import com.gos.lib.rev.swerve.RevSwerveChassis;
import com.gos.lib.rev.swerve.RevSwerveChassisConstants;
import com.gos.lib.rev.swerve.RevSwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;
import org.snobotv2.module_wrappers.phoenix6.Pigeon2Wrapper;

import java.io.IOException;
import java.util.List;

import static edu.wpi.first.units.Units.Degrees;

@SuppressWarnings("PMD.GodClass")
public class ChassisSubsystem extends SubsystemBase {
    private static final double WHEEL_BASE = 0.381;
    private static final double TRACK_WIDTH = 0.381;

    public static final double MAX_TRANSLATION_SPEED;
    public static final double MAX_ROTATION_SPEED = Units.degreesToRadians(720);

    static {
        if (Constants.IS_COMPETITION_ROBOT) {
            // 14p-21s config
            MAX_TRANSLATION_SPEED = 5.4; // Theoretically 6
        } else {
            MAX_TRANSLATION_SPEED = 3.9624; // 13fps, Theoretically 4.8
        }
    }

    private static final GosDoubleProperty ON_THE_FLY_MAX_VELOCITY = new GosDoubleProperty(false, "Chassis On the Fly Max Velocity", 96);
    private static final GosDoubleProperty ON_THE_FLY_MAX_ACCELERATION = new GosDoubleProperty(false, "Chassis On the Fly Max Acceleration", 96);
    private static final GosDoubleProperty ON_THE_FLY_MAX_ANGULAR_VELOCITY = new GosDoubleProperty(false, "Chassis On the Fly Max Angular Velocity", 200);
    private static final GosDoubleProperty ON_THE_FLY_MAX_ANGULAR_ACCELERATION = new GosDoubleProperty(false, "Chassis On the Fly Max Angular Acceleration", 200);

    private static final GosDoubleProperty SLOW_MODE_TRANSLATION_DAMPENING = new GosDoubleProperty(false, "Chassis Slow Mode TranslationJoystickDampening", .5);
    private static final GosDoubleProperty SLOW_MODE_ROTATION_DAMPENING = new GosDoubleProperty(false, "Chassis Slow Mode RotationJoystickDampening", .7);

    private static final GosBooleanProperty USE_APRIL_TAGS = new GosBooleanProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "Chassis: Use AprilTags", true);
    public static final GosDoubleProperty SHOOTER_ARC_CORRECTION = new GosDoubleProperty(false, "Chassis: Shooter Curve Offset", 9);

    private final RevSwerveChassis m_swerveDrive;
    private final Pigeon2 m_gyro;

    private final GoSField m_field;
    private final ValidShootingPolygon m_shootingPolygon;

    private final PIDController m_turnAnglePIDVelocity;
    private final PidProperty m_turnAnglePIDProperties;

    private final AprilTagCameraManager m_aprilTagCameras;
    private final ObjectDetection m_noteDetectionCamera;

    private final LoggingUtil m_logging;

    private boolean m_isSlowTeleop;

    public ChassisSubsystem() {
        m_gyro = new Pigeon2(Constants.PIGEON_PORT);
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());

        m_field = new GoSField();
        SmartDashboard.putData("Field", m_field.getField2d());
        SmartDashboard.putData("Field3d", m_field.getField3d());

        RevSwerveModuleConstants.DriveMotor motorType;
        if (Constants.IS_COMPETITION_ROBOT) {
            motorType = RevSwerveModuleConstants.DriveMotor.VORTEX;
        } else {
            motorType = RevSwerveModuleConstants.DriveMotor.NEO;
        }

        RevSwerveChassisConstants swerveConstants = new RevSwerveChassisConstants(
            Constants.FRONT_LEFT_WHEEL, Constants.FRONT_LEFT_AZIMUTH,
            Constants.BACK_LEFT_WHEEL, Constants.BACK_LEFT_AZIMUTH,
            Constants.FRONT_RIGHT_WHEEL, Constants.FRONT_RIGHT_AZIMUTH,
            Constants.BACK_RIGHT_WHEEL, Constants.BACK_RIGHT_AZIMUTH,
            motorType,
            RevSwerveModuleConstants.DriveMotorPinionTeeth.T14,
            RevSwerveModuleConstants.DriveMotorSpurTeeth.T21,
            WHEEL_BASE,
            TRACK_WIDTH,
            MAX_TRANSLATION_SPEED, MAX_ROTATION_SPEED,
            false);

        m_turnAnglePIDVelocity = new PIDController(0, 0, 0);
        m_turnAnglePIDVelocity.setTolerance(5);
        m_turnAnglePIDVelocity.enableContinuousInput(0, 360);
        m_turnAnglePIDProperties = new WpiPidPropertyBuilder("Chassis to angle", Constants.DEFAULT_CONSTANT_PROPERTIES, m_turnAnglePIDVelocity)
            .addP(0.2)
            .addI(0)
            .addD(0)
            .build();

        m_swerveDrive = new RevSwerveChassis(swerveConstants, m_gyro::getRotation2d, new Pigeon2Wrapper(m_gyro));

        // Cameras
        Matrix<N3, N1> sideSingleTagStddev = AprilTagCamera.DEFAULT_SINGLE_TAG_STDDEV.times(2);
        Matrix<N3, N1> sideMultiTagStddev = AprilTagCamera.DEFAULT_MULTI_TAG_STDDEV.times(2);
        m_aprilTagCameras = new AprilTagCameraManager(FieldConstants.TAG_LAYOUT, List.of(
            new AprilTagCamera(FieldConstants.TAG_LAYOUT, m_field, "Center Back Camera", RobotExtrinsics.ROBOT_TO_CAMERA_APRIL_TAGS_CB),
            new AprilTagCamera(FieldConstants.TAG_LAYOUT, m_field, "Right Camera", RobotExtrinsics.ROBOT_TO_CAMERA_APRIL_TAGS_R, sideSingleTagStddev, sideMultiTagStddev),
            new AprilTagCamera(FieldConstants.TAG_LAYOUT, m_field, "Left Camera", RobotExtrinsics.ROBOT_TO_CAMERA_APRIL_TAGS_L, sideSingleTagStddev, sideMultiTagStddev)
        ));
        m_noteDetectionCamera = new ObjectDetection();

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeed,
            this::setChassisSpeed,
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(10, 0, 0)),
            config,
            GetAllianceUtil::isRedAlliance,
            this
        );

        PathPlannerLogging.setLogActivePathCallback(m_field::setTrajectory);
        PathPlannerLogging.setLogTargetPoseCallback(m_field::setTrajectorySetpoint);

        m_shootingPolygon = new ValidShootingPolygon(m_field);

        m_logging = new LoggingUtil("Chassis");
        m_logging.addDouble("GyroAngle", () -> m_gyro.getYaw().getValue().in(Degrees));
        m_logging.addDouble("PoseAngle", () -> getPose().getRotation().getDegrees());
        m_logging.addDouble("Angle Setpoint", m_turnAnglePIDVelocity::getSetpoint);
        m_logging.addBoolean("At Angle Setpoint", this::isAngleAtGoal);
        m_logging.addDouble("Distance to Speaker", this::getDistanceToSpeaker);
        m_logging.addDouble("Distance to Feeder", () -> getDistanceToFeeder(getPose()));
        m_logging.addBoolean("In Shooting Polygon", this::inShootingPolygon);
        m_logging.addDouble("Chassis X Speed", () -> getChassisSpeed().vxMetersPerSecond);
    }

    public boolean inShootingPolygon() {
        return m_shootingPolygon.containsPoint(getPose().getTranslation());
    }

    public double getDistanceToSpeaker() {
        Pose2d speaker = FieldConstants.Speaker.CENTER_SPEAKER_OPENING.getPose();
        Translation2d roboManTranslation = getPose().getTranslation();
        return roboManTranslation.getDistance(speaker.getTranslation());
    }

    public double getDistanceToAmp() {
        Pose2d amp = RobotExtrinsics.SCORE_IN_AMP_POSITION.getPose();
        Translation2d roboManTranslation = getPose().getTranslation();
        return roboManTranslation.getDistance(amp.getTranslation());
    }

    public double getDistanceToFeeder(Pose2d pose) {
        Pose2d amp = RobotExtrinsics.FULL_FIELD_FEEDING_AIMING_POINT.getPose();
        Translation2d roboManTranslation = getPose().getTranslation();
        return roboManTranslation.getDistance(amp.getTranslation());
    }

    public void resetOdometry(Pose2d pose2d) {
        m_swerveDrive.resetOdometry(pose2d);
    }

    public void setChassisSpeed(ChassisSpeeds speed) {
        m_swerveDrive.setChassisSpeeds(speed);
    }

    public ChassisSpeeds getChassisSpeed() {
        return m_swerveDrive.getChassisSpeed();
    }

    public void clearStickyFaults() {
        m_swerveDrive.clearStickyFaults();
    }

    @Override
    public void periodic() {
        m_swerveDrive.periodic();
        m_logging.updateLogs();

        m_field.setPoseEstimate(m_swerveDrive.getEstimatedPosition());
        m_field.setOdometry(m_swerveDrive.getOdometryPosition());
        m_field.drawNotePoses(m_noteDetectionCamera.objectLocations(getPose()));
        m_turnAnglePIDProperties.updateIfChanged();
        m_field.setFuturePose(getFuturePose(0.3));

        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> estimates = m_aprilTagCameras.update(m_swerveDrive.getEstimatedPosition());

        if (useAprilTagsForPoseEstimation()) {
            for (Pair<EstimatedRobotPose, Matrix<N3, N1>> estimatePair : estimates) {

                EstimatedRobotPose camPose = estimatePair.getFirst();
                Pose2d camEstPose = camPose.estimatedPose.toPose2d();
                m_swerveDrive.addVisionMeasurement(camEstPose, camPose.timestampSeconds, estimatePair.getSecond());
            }
        }
    }

    private boolean useAprilTagsForPoseEstimation() {
        return USE_APRIL_TAGS.getValue();
    }


    @Override
    public void simulationPeriodic() {
        m_swerveDrive.updateSimulator();
        m_aprilTagCameras.updateSimulator(getPose());
        m_noteDetectionCamera.updateObjectDetectionSimulation(getPose());
    }

    public void teleopDrive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
        if (m_isSlowTeleop) {
            m_swerveDrive.driveWithJoysticks(xPercent * SLOW_MODE_TRANSLATION_DAMPENING.getValue(), yPercent * SLOW_MODE_TRANSLATION_DAMPENING.getValue(),
                rotPercent * SLOW_MODE_ROTATION_DAMPENING.getValue(), fieldRelative);
        } else {
            m_swerveDrive.driveWithJoysticks(xPercent, yPercent, rotPercent, fieldRelative);
        }
    }

    public void davidDrive(double x, double y, double angle) {
        if (m_isSlowTeleop) {
            turnToAngleWithVelocity(x * SLOW_MODE_TRANSLATION_DAMPENING.getValue(), y * SLOW_MODE_TRANSLATION_DAMPENING.getValue(),
                angle);
        } else {
            turnToAngleWithVelocity(x, y, angle);
        }
    }

    public Pose2d getPose() {
        return m_swerveDrive.getEstimatedPosition();
    }

    public boolean isAngleAtGoal() {
        return m_turnAnglePIDVelocity.atSetpoint();
    }

    public void turnToAngle(double angleGoal) {
        double angleCurrentDegree = m_swerveDrive.getOdometryPosition().getRotation().getDegrees();
        double steerVelocity = m_turnAnglePIDVelocity.calculate(angleCurrentDegree, angleGoal);

        if (isAngleAtGoal()) {
            steerVelocity = 0;
        }
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, steerVelocity);
        m_swerveDrive.setChassisSpeeds(speeds);
    }

    public void turnToAngleWithVelocity(double xVel, double yVel, double angle) {
        double angleCurrentDegree = getPose().getRotation().getDegrees();
        double steerVelocity = m_turnAnglePIDVelocity.calculate(angleCurrentDegree, angle);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, steerVelocity, getPose().getRotation());


        m_swerveDrive.setChassisSpeeds(speeds);
    }

    public void turnToFacePoint(Pose2d point, double xVel, double yVel) {
        Pose2d robotPose = getPose();
        double xDiff = point.getX() - robotPose.getX();
        double yDiff = point.getY() - robotPose.getY();
        double updateAngle = Math.toDegrees(Math.atan2(yDiff, xDiff));
        turnToAngleWithVelocity(xVel, yVel, updateAngle);
    }

    public void turnButtToFacePoint(Pose2d currentPos, Pose2d endPos, double xVel, double yVel) {
        turnButtToFacePoint(currentPos, endPos, xVel, yVel, SHOOTER_ARC_CORRECTION.getValue());
    }

    public void turnButtToFacePoint(Pose2d currentPos, Pose2d endPos, double xVel, double yVel, double curveOffset) {
        double xDiff = endPos.getX() - currentPos.getX();
        double yDiff = endPos.getY() - currentPos.getY();
        double updateAngle = Math.toDegrees(Math.atan2(yDiff, xDiff));
        updateAngle += 180;
        updateAngle += curveOffset;
        turnToAngleWithVelocity(xVel, yVel, updateAngle);
    }

    public Pose2d getFuturePose() {
        return getFuturePose(.3);
    }

    public Pose2d getFuturePose(double seconds) {
        ChassisSpeeds currentRobotVelocity = m_swerveDrive.getChassisSpeed();
        ChassisSpeeds currentFieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(currentRobotVelocity, getPose().getRotation());
        Pose2d currentPos = m_swerveDrive.getEstimatedPosition();
        Pose2d deltaPos = new Pose2d(
            currentFieldVelocity.vxMetersPerSecond * seconds,
            currentFieldVelocity.vyMetersPerSecond * seconds,
            new Rotation2d(currentFieldVelocity.omegaRadiansPerSecond * seconds));

        return new Pose2d(currentPos.getX() + deltaPos.getX(), currentPos.getY() + deltaPos.getY(), new Rotation2d());
    }

    private void resetGyro() {
        Pose2d currentPose = getPose();
        resetOdometry(new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromDegrees(0)));
    }

    public boolean isNoteDetected() {
        return !m_noteDetectionCamera.objectLocations(getPose()).isEmpty();
    }

    public int numAprilTagsSeen() {
        return m_aprilTagCameras.numAprilTagsSeen();
    }

    public void syncOdometryAndPoseEstimator() {
        m_swerveDrive.resetOdometry(m_swerveDrive.getEstimatedPosition());
    }

    public void takeAprilTagScreenshot() {
        m_aprilTagCameras.takeScreenshot();
    }

    /////////////////////////////////////
    // Checklists
    /////////////////////////////////////


    /////////////////////////////////////
    // Command Factories
    /////////////////////////////////////

    public Command createResetGyroCommand() {
        return run(this::resetGyro)
            .ignoringDisable(true)
            .withName("Reset Gyro");
    }

    public Command createTurnToAngleCommand(double angleGoal) {
        return runOnce(m_turnAnglePIDVelocity::reset)
            .andThen(this.run(() -> turnToAngle(angleGoal))
                .until(this::isAngleAtGoal))
            .withName("Chassis to Angle: " + angleGoal);
    }

    public Command createFollowPathCommand(PathPlannerPath path, boolean resetPose) {
        Command followPathCommand = AutoBuilder.followPath(path);
        if (resetPose) {
            return Commands.runOnce(() -> m_swerveDrive.resetOdometry(path.getStartingDifferentialPose())).andThen(followPathCommand).withName("Reset position and follow path");
        }
        return followPathCommand.withName("Follow Path");
    }

    public Command createDriveToPointNoFlipCommand(Pose2d end) {
        return createDriveToPointNoFlipCommand(end, end.getRotation(), getPose(), false);
    }

    public Command createDriveToPointNoFlipCommand(Pose2d end, Rotation2d endAngle, Pose2d start, boolean rotateFast) {
        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(start, end);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(
                Units.inchesToMeters(ON_THE_FLY_MAX_VELOCITY.getValue()),
                Units.inchesToMeters(ON_THE_FLY_MAX_ACCELERATION.getValue()),
                Units.degreesToRadians(ON_THE_FLY_MAX_ANGULAR_VELOCITY.getValue()),
                Units.degreesToRadians((ON_THE_FLY_MAX_ANGULAR_ACCELERATION.getValue()))),
            null,
            new GoalEndState(0.0, endAngle)
        );
        path.preventFlipping = true;
        return createFollowPathCommand(path, false).withName("Follow Path to " + end);
    }

    public Command createDriveToAmpCommand() {
        return defer(() -> {
            Pose2d ampPosition = new Pose2d(RobotExtrinsics.SCORE_IN_AMP_POSITION.getPose().getTranslation(), Rotation2d.fromDegrees(90));
            Pose2d currentPosition = getPose();
            double dx = ampPosition.getX() - currentPosition.getX();
            double dy = ampPosition.getY() - currentPosition.getY();
            double angle = Math.atan2(dy, dx);
            Pose2d startPose = new Pose2d(currentPosition.getX(), currentPosition.getY(), Rotation2d.fromRadians(angle));
            return createDriveToPointNoFlipCommand(ampPosition, Rotation2d.fromDegrees(-90), startPose, true);
        });
    }

    public Command createPathfindToPoseCommand(MaybeFlippedPose2d pose) {
        return defer(() -> AutoBuilder.pathfindToPose(
            pose.getPose(),

            new PathConstraints(
                Units.inchesToMeters(ON_THE_FLY_MAX_VELOCITY.getValue()),
                Units.inchesToMeters(ON_THE_FLY_MAX_ACCELERATION.getValue()),
                Units.degreesToRadians(ON_THE_FLY_MAX_ANGULAR_VELOCITY.getValue()),
                Units.degreesToRadians((ON_THE_FLY_MAX_ANGULAR_ACCELERATION.getValue()))),
            0.0 // Goal end velocity in meters/sec
        ));

    }

    public Command createDriveToPointCommand(Pose2d endPoint) {
        return createDriveToPointNoFlipCommand(endPoint).withName("Drive to " + endPoint);
    }

    public Command createResetPoseCommand(MaybeFlippedPose2d pose) {
        return defer(() -> createResetPoseCommand(pose.getPose()));
    }

    public Command createResetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetOdometry(pose))
            .ignoringDisable(true)
            .withName("Reset Pose " + pose);
    }

    public Command createPushForwardModeCommand() {
        return runEnd(() -> m_swerveDrive.setModulesToPushMode(0), m_swerveDrive::setModuleBrakeMode)
            .ignoringDisable(true)
            .withName("Chassis Push Forward");
    }

    public Command createPushSidewaysModeCommand() {
        return runEnd(() -> m_swerveDrive.setModulesToPushMode(90), m_swerveDrive::setModuleBrakeMode)
            .ignoringDisable(true)
            .withName("Chassis Push Sideways");
    }

    public Command createSetSlowModeCommand(boolean setBoolean) {
        return runOnce(() -> m_isSlowTeleop = setBoolean);
    }

    public Command createDriveToNoteCommand() {
        return defer(() -> {
            List<Pose2d> notePositions = m_noteDetectionCamera.objectLocations(getPose());
            if (notePositions.isEmpty()) {
                return Commands.none();
            }
            Pose2d singleNote = notePositions.get(0);

            Pose2d currentPosition = getPose();
            Translation2d deltaTranslation = singleNote.getTranslation().minus(currentPosition.getTranslation());
            Rotation2d deltaAngle = deltaTranslation.getAngle();
            Pose2d startPose = new Pose2d(currentPosition.getX(), currentPosition.getY(), deltaAngle);
            return createDriveToPointNoFlipCommand(singleNote, deltaAngle, startPose, true);
        }).withName("drive to note");
    }

    // This command will reset the pose and believe the april tag estimation, regardless of mode or filtering
    public Command createBelieveAprilTagEstimatorCommand() {
        return run(() -> {
            Pose3d firstEstimatedPose = m_aprilTagCameras.getFirstEstimatedPose();
            if (firstEstimatedPose != null) {
                m_swerveDrive.resetOdometry(firstEstimatedPose.toPose2d());
            }
        }).withTimeout(.1).ignoringDisable(true).withName("Believe AprilTags");
    }

    public Command createSyncOdometryAndPoseEstimatorCommand() {
        return run(this::syncOdometryAndPoseEstimator).ignoringDisable(true);
    }

    public Command createTakeAprilTagScreenshotCommand() {
        // This is an instant command instead of run/runEnd because we don't want the "requirement" logic on the chassis to happen
        return new InstantCommand(this::takeAprilTagScreenshot);
    }

    public Command createSetChassisSpeedCommand(ChassisSpeeds chassisSpeeds) {
        return run(() -> setChassisSpeed(chassisSpeeds));
    }
}
