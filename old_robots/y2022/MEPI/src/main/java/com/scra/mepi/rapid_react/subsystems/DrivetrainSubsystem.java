// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.scra.mepi.rapid_react.subsystems;

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.scra.mepi.rapid_react.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.json.simple.parser.ParseException;
import org.snobotv2.module_wrappers.navx.NavxWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

import java.io.IOException;

// Drive train
@SuppressWarnings({"PMD.TooManyMethods", "PMD.TooManyFields"})
public class DrivetrainSubsystem extends SubsystemBase {
    private final SparkMax m_leftLeader =
        new SparkMax(Constants.DRIVE_LEFT_LEADER, MotorType.kBrushless);
    private final SparkMax m_leftFollower =
        new SparkMax(Constants.DRIVE_LEFT_FOLLOWER, MotorType.kBrushless);
    private final SparkMax m_rightLeader =
        new SparkMax(Constants.DRIVE_RIGHT_LEADER, MotorType.kBrushless);
    private final SparkMax m_rightFollower =
        new SparkMax(Constants.DRIVE_RIGHT_FOLLOWER, MotorType.kBrushless);
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
    private final DifferentialDriveKinematics m_kinematics =
        new DifferentialDriveKinematics(Constants.DRIVE_TRACK);

    // odometry
    private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    private final Field2d m_field = new Field2d();

    // PID
    private final SparkClosedLoopController m_leftController = m_leftLeader.getClosedLoopController();
    private final PidProperty m_leftProperties;
    private final SparkClosedLoopController m_rightController = m_rightLeader.getClosedLoopController();
    private final PidProperty m_rightProperties;

    // Simulation
    private DifferentialDrivetrainSimWrapper m_simulator;

    /**
     * Creates a new DrivetrainSubsystem.
     */
    public DrivetrainSubsystem() {
        SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

        leftLeaderConfig.smartCurrentLimit(50);
        leftFollowerConfig.smartCurrentLimit(50);
        rightLeaderConfig.smartCurrentLimit(50);
        rightFollowerConfig.smartCurrentLimit(50);

        m_leftLeader.setInverted(true);

        leftFollowerConfig.follow(m_leftLeader);
        rightFollowerConfig.follow(m_rightLeader);

        m_leftProperties = setupVelocityPidValues(m_leftLeader, leftLeaderConfig);
        m_rightProperties = setupVelocityPidValues(m_rightLeader, rightLeaderConfig);

        leftLeaderConfig.encoder.positionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
        rightLeaderConfig.encoder.positionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
        leftLeaderConfig.encoder.velocityConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
        rightLeaderConfig.encoder.velocityConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);

        m_leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putData(m_field);


        if (RobotBase.isSimulation()) {
            DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
                DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                DifferentialDrivetrainSim.KitbotGearing.k5p95,
                DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                null);
            m_simulator = new DifferentialDrivetrainSimWrapper(
                drivetrainSim,
                new RevMotorControllerSimWrapper(m_leftLeader),
                new RevMotorControllerSimWrapper(m_rightLeader),
                RevEncoderSimWrapper.create(m_leftLeader),
                RevEncoderSimWrapper.create(m_rightLeader),
                new NavxWrapper().getYawGyro());
            m_simulator.setRightInverted(false);
        }
    }

    private PidProperty setupVelocityPidValues(SparkMax motor, SparkMaxConfig config) {
        return new RevPidPropertyBuilder("ChassisVelocity", false, motor, config, ClosedLoopSlot.kSlot0)
            .addP(1)
            .addI(0)
            .addD(0)
            .build();
    }

    public void control(double speed, double rotation) {
        m_drive.curvatureDrive(speed, rotation, Math.abs(speed) < 0.05);
    }

    public void smartVelocityControl(double leftVelocity, double rightVelocity) {
        m_leftController.setReference(leftVelocity, ControlType.kVelocity);
        m_rightController.setReference(rightVelocity, ControlType.kVelocity);
    }

    public void applyChassisSpeed(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        applyWheelSpeed(wheelSpeeds);
    }

    public void applyWheelSpeed(DifferentialDriveWheelSpeeds speeds) {
        m_leftController.setReference(speeds.leftMetersPerSecond, ControlType.kSmartVelocity);
        m_rightController.setReference(speeds.rightMetersPerSecond, ControlType.kSmartVelocity);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public double leftVelocity() {
        return m_leftEncoder.getVelocity();
    }

    public double rightVelocity() {
        return m_rightEncoder.getVelocity();
    }

    public Command createPathFollowingCommand(String pathName) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
        m_odometry.update(gyroAngle, m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
        m_field.setRobotPose(m_odometry.getPoseMeters());
        SmartDashboard.putNumber("Chassis Left Velocity", leftVelocity());
        SmartDashboard.putNumber("Chassis Right Velocity", rightVelocity());

        m_leftProperties.updateIfChanged();
        m_rightProperties.updateIfChanged();
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
    }
}
