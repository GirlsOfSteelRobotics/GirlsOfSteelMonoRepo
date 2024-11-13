package com.gos.infinite_recharge.subsystems;

import com.gos.infinite_recharge.Constants;
import com.gos.infinite_recharge.Constants.DriveConstants;
import com.gos.infinite_recharge.sim.CameraSimulator;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import org.snobotv2.coordinate_gui.RobotPositionPublisher;
import org.snobotv2.module_wrappers.navx.NavxWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N2;

import com.revrobotics.spark.SparkClosedLoopController;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("PMD.TooManyFields")
public class Chassis extends SubsystemBase {

    //private static final double FULL_THROTTLE_SECONDS = 1.0;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final double GEAR_RATIO = 40.0 / 10.0 * 34.0 / 20.0;
    private static final double ENCODER_CONSTANT = (1.0 / GEAR_RATIO) * WHEEL_DIAMETER * Math.PI;

    private final SparkMax m_masterLeft;
    private final SparkMax m_followerLeft; // NOPMD

    private final SparkMax m_masterRight;
    private final SparkMax m_followerRight; // NOPMD

    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;

    private final SparkClosedLoopController m_leftPidController;
    private final SparkClosedLoopController m_rightPidController;

    private final AHRS m_gyro;

    private final DifferentialDrive m_drive;

    private final DifferentialDriveOdometry m_odometry;
    private final List<Field2d> m_fields;

    private final RobotPositionPublisher m_coordinateGuiPublisher;

    private final PidProperty m_leftProperties;
    private final PidProperty m_rightProperties;

    private DifferentialDrivetrainSimWrapper m_simulator;
    private CameraSimulator m_cameraSimulator;

    @SuppressWarnings({"PMD.NcssCount", "PMD.ExcessiveMethodLength"})
    public Chassis() {
        m_masterLeft = new SparkMax(Constants.DRIVE_LEFT_MASTER_SPARK, MotorType.kBrushless);
        m_followerLeft = new SparkMax(Constants.DRIVE_LEFT_FOLLOWER_SPARK, MotorType.kBrushless);
        m_masterRight = new SparkMax(Constants.DRIVE_RIGHT_MASTER_SPARK, MotorType.kBrushless);
        m_followerRight = new SparkMax(Constants.DRIVE_RIGHT_FOLLOWER_SPARK, MotorType.kBrushless);

        SparkMaxConfig masterLeftConfig = new SparkMaxConfig();
        SparkMaxConfig followerLeftConfig = new SparkMaxConfig();
        SparkMaxConfig masterRightConfig = new SparkMaxConfig();
        SparkMaxConfig followerRightConfig = new SparkMaxConfig();

        m_rightEncoder = m_masterRight.getEncoder();
        m_leftEncoder = m_masterLeft.getEncoder();

        m_leftPidController = m_masterLeft.getClosedLoopController();
        m_rightPidController = m_masterRight.getClosedLoopController();

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

        masterLeftConfig.encoder.positionConversionFactor(ENCODER_CONSTANT);
        masterRightConfig.encoder.positionConversionFactor(ENCODER_CONSTANT);

        masterLeftConfig.encoder.velocityConversionFactor(ENCODER_CONSTANT / 60.0);
        masterRightConfig.encoder.velocityConversionFactor(ENCODER_CONSTANT / 60.0);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

        m_gyro = new AHRS();

        IdleMode idleMode = IdleMode.kCoast;
        masterLeftConfig.idleMode(idleMode);
        followerLeftConfig.idleMode(idleMode);
        masterRightConfig.idleMode(idleMode);
        followerRightConfig.idleMode(idleMode);

        m_masterLeft.setInverted(false);
        m_masterRight.setInverted(true);

        followerLeftConfig.follow(m_masterLeft, false);
        followerRightConfig.follow(m_masterRight, false);

        masterLeftConfig.smartCurrentLimit(Constants.SPARK_MAX_CURRENT_LIMIT);
        followerLeftConfig.smartCurrentLimit(Constants.SPARK_MAX_CURRENT_LIMIT);
        masterRightConfig.smartCurrentLimit(Constants.SPARK_MAX_CURRENT_LIMIT);
        followerRightConfig.smartCurrentLimit(Constants.SPARK_MAX_CURRENT_LIMIT);

        // m_masterLeft.setOpenLoopRampRate(FULL_THROTTLE_SECONDS);
        // m_masterRight.setOpenLoopRampRate(FULL_THROTTLE_SECONDS);

        m_drive = new DifferentialDrive(m_masterLeft, m_masterRight);
        m_drive.setSafetyEnabled(true);
        m_drive.setExpiration(0.1);
        m_drive.setMaxOutput(0.8);

        m_coordinateGuiPublisher = new RobotPositionPublisher();

        // Smart Motion stuff
        double kp = 0.01000;
        double ki = 0;
        double kd = 0;
        double kff = 0.005800;
        boolean lockConstants = false;
        double minVel = 0; // inch/sec
        double maxVel = 72; // inch/sec
        double maxAcc = 144; // inch/sec/sec
        double allowedErr = 0;
        double kMaxOutput = 1;
        double kMinOutput = -1;
        ClosedLoopSlot smartMotionSlot = ClosedLoopSlot.kSlot0;

        m_leftProperties = new RevPidPropertyBuilder("Chassis", lockConstants, m_leftPidController, 0)
            .addP(kp)
            .addI(ki)
            .addD(kd)
            .addFF(kff)
            .addMaxVelocity(maxVel)
            .addMaxAcceleration(maxAcc)
            .build();
        m_rightProperties = new RevPidPropertyBuilder("Chassis", lockConstants, m_rightPidController, 0)
            .addP(kp)
            .addI(ki)
            .addD(kd)
            .addFF(kff)
            .addMaxVelocity(maxVel)
            .addMaxAcceleration(maxAcc)
            .build();

        masterLeftConfig.closedLoop.outputRange(kMinOutput, kMaxOutput);
        masterRightConfig.closedLoop.outputRange(kMinOutput, kMaxOutput);
        masterLeftConfig.closedLoop.smartMotion.allowedClosedLoopError(allowedErr, smartMotionSlot);
        masterRightConfig.closedLoop.smartMotion.allowedClosedLoopError(allowedErr, smartMotionSlot);

        masterLeftConfig.closedLoop.smartMotion.minOutputVelocity(minVel, smartMotionSlot);
        masterRightConfig.closedLoop.smartMotion.minOutputVelocity(minVel, smartMotionSlot);

        m_leftProperties.updateIfChanged(true);
        m_rightProperties.updateIfChanged(true);

        m_masterLeft.configure(masterLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerLeft.configure(followerLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_masterRight.configure(masterRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerRight.configure(followerRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_drive.setSafetyEnabled(false);

        m_fields = new ArrayList<>();
        createField("InfiniteRecharge");
        createField("Barrel");
        createField("Bounce");
        createField("GalacticSearchA");
        createField("GalacticSearchB");
        createField("Slalom");


        if (RobotBase.isSimulation()) {

            LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
                    DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER,
                    DriveConstants.KV_VOLT_SECONDS_PER_RADIAN, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_RADIAN);
            DCMotor kDriveGearbox = DCMotor.getNEO(2);

            DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
                    kDrivetrainPlant,
                    kDriveGearbox,
                    GEAR_RATIO,
                    DriveConstants.TRACK_WIDTH_METERS,
                    Units.inchesToMeters(WHEEL_DIAMETER) / 2.0,
                    null);

            m_simulator = new DifferentialDrivetrainSimWrapper(
                    drivetrainSim,
                    new RevMotorControllerSimWrapper(m_masterLeft),
                    new RevMotorControllerSimWrapper(m_masterRight),
                    RevEncoderSimWrapper.create(m_masterLeft),
                    RevEncoderSimWrapper.create(m_masterRight),
                    new NavxWrapper().getYawGyro());
            m_simulator.setRightInverted(false);
            m_cameraSimulator = new CameraSimulator();
        }
    }

    private void createField(String name) {

        Field2d field = new Field2d(); // NOPMD(CloseResource)
        m_fields.add(field);
        Shuffleboard.getTab("Fields").add(name + "Field", field);
    }

    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), Units.inchesToMeters(m_leftEncoder.getPosition()),
            Units.inchesToMeters(m_rightEncoder.getPosition()));

        for (Field2d field : m_fields) { // NOPMD(CloseResource)
            field.setRobotPose(m_odometry.getPoseMeters());
        }

        // SmartDashboard.putNumber("x", getX());
        // SmartDashboard.putNumber("y", getY());
        // SmartDashboard.putNumber("yaw", getHeading());
        // SmartDashboard.putNumber("right encoder", getRightEncoder());
        // SmartDashboard.putNumber("left encoder", getLeftEncoder());
        SmartDashboard.putNumber("right encoder speed", getRightEncoderSpeed());
        SmartDashboard.putNumber("left encoder speed", getLeftEncoderSpeed());

        m_coordinateGuiPublisher.publish(getPose());

        m_leftProperties.updateIfChanged();
        m_rightProperties.updateIfChanged();
    }

    //////////////////////////////
    // Odometry Stuff
    //////////////////////////////
    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    }

    public double getRightEncoder() {
        return m_rightEncoder.getPosition();
    }

    public double getLeftEncoderSpeed() {
        return m_leftEncoder.getVelocity();
    }

    public double getRightEncoderSpeed() {
        return m_rightEncoder.getVelocity();
    }

    public void setPositionInches(double x, double y, double angle) {
        setPositionMeters(Units.inchesToMeters(x), Units.inchesToMeters(y), angle);
    }

    public void setPositionMeters(double x, double y, double angle) {
        resetOdometry(new Pose2d(x, y, Rotation2d.fromDegrees(angle)));
    }

    public void resetOdometry(Pose2d pose) {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
        m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
        m_simulator.resetOdometry(pose);
    }

    public double getXMeters() {
        return m_odometry.getPoseMeters().getTranslation().getX();
    }

    public double getYMeters() {
        return m_odometry.getPoseMeters().getTranslation().getY();
    }

    public double getXInches() {
        return Units.metersToInches(getXMeters());
    }

    public double getYInches() {
        return Units.metersToInches(getYMeters());
    }

    public double getHeading() {
        return -m_gyro.getAngle();
        // return 0;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
    }

    //////////////////////////////
    // Functions to make it drive
    //////////////////////////////
    public void setSpeed(final double speed) {
        // System.out.println("SETTING SPEED");
        m_drive.arcadeDrive(speed, 0);
    }

    // command to rotate robot to align with target based on limelight value
    public void setSteer(double steer) {
        m_drive.arcadeDrive(0, steer);
        // System.out.println("SETTING STEER");
    }

    public void setSpeedAndSteer(double speed, double steer) {
        m_drive.arcadeDrive(speed, steer);
    }

    public void driveDistance(double leftPosition, double rightPosition) {
        m_leftPidController.setReference(leftPosition, ControlType.kSmartMotion);
        m_rightPidController.setReference(rightPosition, ControlType.kSmartMotion);
        m_drive.feed();
    }

    public void smartVelocityControl(double leftVelocity, double rightVelocity) {
        // System.out.println("Driving velocity");
        m_leftPidController.setReference(leftVelocity, ControlType.kVelocity);
        m_rightPidController.setReference(rightVelocity, ControlType.kVelocity);
        m_drive.feed();

        //System.out.println("Left Velocity" + leftVelocity + ", Right Velocity" + rightVelocity);
    }

    public void smartVelocityControlMeters(double leftVelocityMeters, double rightVelocityMeters) {
        double leftVelocityInchesPerSecond = Units.metersToInches(leftVelocityMeters);
        double rightVelocityInchesPerSecond = Units.metersToInches(rightVelocityMeters);

        smartVelocityControl(leftVelocityInchesPerSecond, rightVelocityInchesPerSecond);
    }

    public void stop() {
        m_drive.stopMotor();
        System.out.println("Stopping motors");
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
        m_cameraSimulator.update(getPose());
    }
}
