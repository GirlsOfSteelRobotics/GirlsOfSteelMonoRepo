package com.gos.chargedup.subsystems;

import com.gos.chargedup.Constants;
import com.gos.lib.GetAllianceUtil;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.gos.lib.rev.checklists.SparkMaxMotorsMoveChecklist;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.revrobotics.spark.SparkBase.ControlType;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;
import org.snobotv2.module_wrappers.phoenix6.Pigeon2Wrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;


@SuppressWarnings("PMD.GodClass")
public class TankDriveChassisSubsystem extends BaseChassis implements ChassisSubsystemInterface {
    private static final GosDoubleProperty AUTO_ENGAGE_KP = new GosDoubleProperty(false, "Chassis auto engage kP", .025);

    private static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0);
    private static final double GEAR_RATIO;

    static {
        if (Constants.IS_ROBOT_BLOSSOM) {
            GEAR_RATIO = 527.0 / 54.0;
        } else {
            GEAR_RATIO = 40.0 / 12.0 * 40.0 / 14.0;
        }
    }

    private static final double ENCODER_CONSTANT = (1.0 / GEAR_RATIO) * WHEEL_DIAMETER * Math.PI;

    private static final double TRACK_WIDTH = 0.381 * 2; //set this to the actual
    public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS =
        new DifferentialDriveKinematics(TRACK_WIDTH);
    public static final double KS_VOLTS_STATIC_FRICTION_TURNING = 0;


    //Chassis and motors
    private final SparkMax m_leaderLeft;
    private final SparkMax m_followerLeft;
    private final SparkMax m_leaderRight;
    private final SparkMax m_followerRight;

    private final DifferentialDrive m_drive;

    //Odometry
    private final DifferentialDriveOdometry m_odometry;
    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;

    //SIM
    private DifferentialDrivetrainSimWrapper m_simulator;

    private final DifferentialDrivePoseEstimator m_poseEstimator;

    private final SparkClosedLoopController m_leftPIDcontroller;
    private final SparkClosedLoopController m_rightPIDcontroller;

    private final PidProperty m_leftPIDProperties;
    private final PidProperty m_rightPIDProperties;

    private final GosDoubleProperty m_maxVelocity = new GosDoubleProperty(true, "Chassis Trajectory Max Velocity", 60);

    private final SparkMaxAlerts m_leaderLeftMotorErrorAlert;
    private final SparkMaxAlerts m_followerLeftMotorErrorAlert;
    private final SparkMaxAlerts m_leaderRightMotorErrorAlert;
    private final SparkMaxAlerts m_followerRightMotorErrorAlert;

    //max velocity and acceleration tuning
    private final GosDoubleProperty m_onTheFlyMaxVelocity = new GosDoubleProperty(true, "Chassis On the Fly Max Acceleration", 48);
    private final GosDoubleProperty m_onTheFlyMaxAcceleration = new GosDoubleProperty(true, "Chassis On the Fly Max Acceleration", 48);

    @SuppressWarnings({"PMD.NcssCount", "PMD.ExcessiveMethodLength"})
    public TankDriveChassisSubsystem() {

        m_leaderLeft = new SparkMax(Constants.DRIVE_LEFT_LEADER_SPARK, MotorType.kBrushless);
        m_followerLeft = new SparkMax(Constants.DRIVE_LEFT_FOLLOWER_SPARK, MotorType.kBrushless);
        m_leaderRight = new SparkMax(Constants.DRIVE_RIGHT_LEADER_SPARK, MotorType.kBrushless);
        m_followerRight = new SparkMax(Constants.DRIVE_RIGHT_FOLLOWER_SPARK, MotorType.kBrushless);

        SparkMaxConfig leaderLeftConfig = new SparkMaxConfig();
        SparkMaxConfig followerLeftConfig = new SparkMaxConfig();
        SparkMaxConfig leaderRightConfig = new SparkMaxConfig();
        SparkMaxConfig followerRightConfig = new SparkMaxConfig();

        leaderLeftConfig.smartCurrentLimit(60);
        followerLeftConfig.smartCurrentLimit(60);
        leaderRightConfig.smartCurrentLimit(60);
        followerRightConfig.smartCurrentLimit(60);

        leaderLeftConfig.idleMode(IdleMode.kCoast);
        followerLeftConfig.idleMode(IdleMode.kCoast);
        leaderRightConfig.idleMode(IdleMode.kCoast);
        followerRightConfig.idleMode(IdleMode.kCoast);

        if (Constants.IS_ROBOT_BLOSSOM) {
            m_leaderLeft.setInverted(true);
            m_leaderRight.setInverted(false);
        }
        else {
            m_leaderLeft.setInverted(false);
            m_leaderRight.setInverted(true);
        }

        followerLeftConfig.follow(m_leaderLeft, false);
        followerRightConfig.follow(m_leaderRight, false);

        m_drive = new DifferentialDrive(m_leaderLeft, m_leaderRight);


        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

        m_leftPIDcontroller = m_leaderLeft.getClosedLoopController();
        m_rightPIDcontroller = m_leaderRight.getClosedLoopController();

        m_leftPIDProperties = setupPidValues(m_leftPIDcontroller);
        m_rightPIDProperties = setupPidValues(m_rightPIDcontroller);

        m_rightEncoder = m_leaderRight.getEncoder();
        m_leftEncoder = m_leaderLeft.getEncoder();

        leaderLeftConfig.encoder.positionConversionFactor(ENCODER_CONSTANT);
        leaderRightConfig.encoder.positionConversionFactor(ENCODER_CONSTANT);

        leaderLeftConfig.encoder.velocityConversionFactor(ENCODER_CONSTANT / 60.0);
        leaderRightConfig.encoder.velocityConversionFactor(ENCODER_CONSTANT / 60.0);

        m_leaderLeft.configure(leaderLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerLeft.configure(followerLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leaderRight.configure(leaderRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerRight.configure(followerRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_poseEstimator = new DifferentialDrivePoseEstimator(
            K_DRIVE_KINEMATICS, m_gyro.getRotation2d(), 0.0, 0.0, new Pose2d());

        m_networkTableEntries.addDouble("Left Position", () -> Units.metersToInches(m_leftEncoder.getPosition()));
        m_networkTableEntries.addDouble("Left Velocity", () -> Units.metersToInches(m_leftEncoder.getVelocity()));
        m_networkTableEntries.addDouble("Right Position", () -> Units.metersToInches(m_rightEncoder.getPosition()));
        m_networkTableEntries.addDouble("Rigth Velocity", () -> Units.metersToInches(m_rightEncoder.getVelocity()));

        m_leaderLeftMotorErrorAlert = new SparkMaxAlerts(m_leaderLeft, "left chassis motor ");
        m_followerLeftMotorErrorAlert = new SparkMaxAlerts(m_followerLeft, "left chassis motor ");
        m_leaderRightMotorErrorAlert = new SparkMaxAlerts(m_leaderRight, "right chassis motor ");
        m_followerRightMotorErrorAlert = new SparkMaxAlerts(m_followerRight, "right chassis motor ");


        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry,
            this::getChassisSpeed,
            this::setChassisSpeed,
            new PPLTVController(0.02),
            config,
            GetAllianceUtil::isRedAlliance,
            this
        );

        if (RobotBase.isSimulation()) {
            DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
                DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                DifferentialDrivetrainSim.KitbotGearing.k5p95,
                DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                null);
            m_simulator = new DifferentialDrivetrainSimWrapper(
                drivetrainSim,
                new RevMotorControllerSimWrapper(m_leaderLeft),
                new RevMotorControllerSimWrapper(m_leaderRight),
                RevEncoderSimWrapper.create(m_leaderLeft),
                RevEncoderSimWrapper.create(m_leaderRight),
                new Pigeon2Wrapper(m_gyro));
            m_simulator.setRightInverted(false);
        }
    }

    private PidProperty setupPidValues(SparkClosedLoopController pidController) {
        if (Constants.IS_ROBOT_BLOSSOM) {
            return new RevPidPropertyBuilder("Chassis", true, pidController, 0)
                .addP(0.6)
                .addI(0)
                .addD(0)
                .addFF(0.215)
                // .addMaxVelocity(0)
                // .addMaxAcceleration(0)
                .build();
        }
        else {
            return new RevPidPropertyBuilder("Chassis", false, pidController, 0)
                .addP(0)
                .addI(0)
                .addD(0)
                .addFF(.22)
                // .addMaxVelocity(2)
                // .addMaxAcceleration(0)
                .build();
        }
    }

    @Override
    public void periodic() {

        updateOdometry();

        m_field.setOdometry(m_odometry.getPoseMeters());
        m_field.setPoseEstimate(m_poseEstimator.getEstimatedPosition());

        m_leftPIDProperties.updateIfChanged();
        m_rightPIDProperties.updateIfChanged();
        m_turnAnglePIDProperties.updateIfChanged();
        m_turnAnglePIDFFProperty.updateIfChanged();
        m_turnPidAllowableError.updateIfChanged();

        m_leaderLeftMotorErrorAlert.checkAlerts();
        m_followerLeftMotorErrorAlert.checkAlerts();
        m_leaderRightMotorErrorAlert.checkAlerts();
        m_followerRightMotorErrorAlert.checkAlerts();
        m_pigeonAlerts.checkAlerts();
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
    }

    @Override
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void smartVelocityControl(double leftVelocity, double rightVelocity) {
        m_leftPIDcontroller.setReference(leftVelocity, ControlType.kVelocity, 0);
        m_rightPIDcontroller.setReference(rightVelocity, ControlType.kVelocity, 0);
    }

    @Override
    public void stop() {
        setArcadeDrive(0, 0);
    }

    public void setArcadeDrive(double speed, double steer) {
        m_drive.arcadeDrive(speed, steer);
    }

    public void setCurvatureDrive(double speed, double steer, boolean allowTurnInPlace) {
        m_drive.curvatureDrive(speed, steer, allowTurnInPlace);
    }

    @Override
    public void turnToAngle(double angleGoal) {
        SmartDashboard.putNumber("goal angle chassis pid", angleGoal);
        double steerVoltage = m_turnAnglePID.calculate(m_odometry.getPoseMeters().getRotation().getDegrees(), angleGoal);
        AngularVelocity currentVelocity = DegreesPerSecond.of(m_gyro.getRate());
        AngularVelocity goalVelocity = DegreesPerSecond.of(m_turnAnglePID.getSetpoint().velocity);
        steerVoltage += m_turnAnglePIDFFProperty.calculate(currentVelocity, goalVelocity).in(Volts);

        m_gyroAngleGoalVelocityEntry.setNumber(m_turnAnglePID.getSetpoint().velocity);


        if (turnPIDIsAtAngle()) {
            steerVoltage = 0;
        }

        m_leaderRight.setVoltage(steerVoltage);
        m_leaderLeft.setVoltage(-steerVoltage);
        m_drive.feed();
    }

    @Override
    public void autoEngage() {
        double speed;
        if (Constants.IS_ROBOT_BLOSSOM) {
            speed = getPitch() * AUTO_ENGAGE_KP.getValue();
        }
        else {
            speed = -getPitch() * AUTO_ENGAGE_KP.getValue();
        }

        if (getPitch() > PITCH_LOWER_LIMIT && getPitch() < PITCH_UPPER_LIMIT) {
            setArcadeDrive(0, 0);

        } else if (getPitch() > 0) {
            if (speed < -0.35) {
                speed = -0.35;
            }
            setArcadeDrive(speed, 0);

        } else if (getPitch() < 0) {
            if (speed > 0.35) {
                speed = 0.35;
            }
            setArcadeDrive(speed, 0);
        }
        m_tryingToEngage = true;
    }



    private void updateOdometry() {
        m_poseEstimator.update(
            m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        if (DriverStation.isTeleop()) {
            for (Vision vision : m_cameras) {
                updateCameraEstimate(vision);
            }
        }
    }

    private void updateCameraEstimate(Vision vision) {
        //NEW ODOMETRY
        Optional<EstimatedRobotPose> result =
            vision.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            Pose2d pose2d = camPose.estimatedPose.toPose2d();
            m_poseEstimator.addVisionMeasurement(pose2d, camPose.timestampSeconds);
        }
    }

    @Override
    public void resetOdometry(Pose2d pose2d) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose2d);
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose2d);
        //System.out.println("Reset Odometry was called");
    }

    @Override
    public void clearStickyFaults() {
        m_leaderLeft.clearFaults();
        m_leaderRight.clearFaults();
        m_followerLeft.clearFaults();
        m_followerRight.clearFaults();
    }

    @Override
    public Command createFollowPathCommand(PathPlannerPath path, boolean resetPose) {
        Command followPathCommand = AutoBuilder.followPath(path);
        if (resetPose) {
            return Commands.runOnce(() -> resetOdometry(path.getStartingDifferentialPose())).andThen(followPathCommand);
        }
        return followPathCommand;
    }

    @Override
    public Command createDriveToPointNoFlipCommand(Pose2d start, Pose2d end, boolean reverse) {
        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(start, end);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(m_onTheFlyMaxVelocity.getValue(), m_onTheFlyMaxAcceleration.getValue(), 0, 0),
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return createFollowPathCommand(path, false);
    }

    public void drivetrainToBrakeMode() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        m_leaderLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followerLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_leaderRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followerRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void drivetrainToCoastMode() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        m_leaderLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followerLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_leaderRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followerRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    protected void lockDriveTrain() {
        drivetrainToBrakeMode();
    }

    @Override
    protected void unlockDriveTrain() {
        drivetrainToCoastMode();
    }

    @Override
    public ChassisSpeeds getChassisSpeed() {
        return K_DRIVE_KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity()));
    }

    @Override
    public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds diffSpeeds = K_DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds);
        smartVelocityControl(diffSpeeds.leftMetersPerSecond, diffSpeeds.rightMetersPerSecond);
    }

    ////////////////////
    // Command Factories
    ////////////////////

    public Command commandChassisVelocity() {
        return this.runEnd(
            () -> smartVelocityControl(Units.inchesToMeters(m_maxVelocity.getValue()), Units.inchesToMeters(m_maxVelocity.getValue())),
            this::stop).withName("Chassis: Tune Velocity");
    }


    @Override
    public Command createSyncOdometryWithPoseEstimatorCommand() {
        return runOnce(() ->  m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), m_poseEstimator.getEstimatedPosition()))
            .withName("Sync Odometry /w Pose");
    }

    @Override
    public Command createSelfTestMotorsCommand() {
        return new SequentialCommandGroup(createIsLeftMotorMoving(), createIsRightMotorMoving());
    }

    ////////////////
    // Checklists
    ////////////////
    public Command createIsLeftMotorMoving() {
        return new SparkMaxMotorsMoveChecklist(this, m_leaderLeft, "Chassis: Leader left motor", 1.0);
    }

    public Command createIsRightMotorMoving() {
        return new SparkMaxMotorsMoveChecklist(this, m_leaderRight, "Chassis: Leader right motor", 1.0);
    }



}
