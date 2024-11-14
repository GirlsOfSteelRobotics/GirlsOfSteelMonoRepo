package com.gos.codelabs.pid.subsystems;

import com.gos.codelabs.pid.Constants;
import com.gos.codelabs.pid.Constants.DrivetrainConstants;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.module_wrappers.wpi.ADXRS450GyroWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

@SuppressWarnings("PMD.TooManyFields")
public class ChassisSubsystem extends SubsystemBase {

    private static final ClosedLoopSlot PID_SLOT_VELOCITY = ClosedLoopSlot.kSlot0;
    private static final ClosedLoopSlot PID_SLOT_SMART_MOTION = ClosedLoopSlot.kSlot1;
    private static final ClosedLoopSlot PID_SLOT_POSITION = ClosedLoopSlot.kSlot2;

    public static final double DEFAULT_ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(.5);

    private final SparkMax m_leftDriveA;
    private final SparkMax m_rightDriveA;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;
    private final SparkClosedLoopController m_leftPid;
    private final SparkClosedLoopController m_rightPid;
    private final PidProperty m_leftVelocityPidProperty;
    private final PidProperty m_rightVelocityPidProperty;
    private final PidProperty m_leftSmartMotionPidProperty;
    private final PidProperty m_rightSmartMotionPidProperty;
    private final PidProperty m_leftPositionPidProperty;
    private final PidProperty m_rightPositionPidProperty;

    private final DifferentialDrive m_differentialDrive;
    private final DifferentialDriveOdometry m_odometry;
    private final Field2d m_field;

    private final ADXRS450_Gyro m_gyro;

    private DifferentialDrivetrainSimWrapper m_simulator;

    @SuppressWarnings("PMD.CloseResource")
    public ChassisSubsystem() {

        m_leftDriveA = new SparkMax(Constants.CAN_CHASSIS_LEFT_A, MotorType.kBrushless);
        SparkMax leftDriveB = new SparkMax(Constants.CAN_CHASSIS_LEFT_B, MotorType.kBrushless);

        SparkMaxConfig leftDriveAConfig = new SparkMaxConfig();
        SparkMaxConfig leftDriveBConfig = new SparkMaxConfig();
        leftDriveBConfig.follow(m_leftDriveA);

        m_rightDriveA = new SparkMax(Constants.CAN_CHASSIS_RIGHT_A, MotorType.kBrushless);
        SparkMax rightDriveB = new SparkMax(Constants.CAN_CHASSIS_RIGHT_B, MotorType.kBrushless);
        SparkMaxConfig rightDriveAConfig = new SparkMaxConfig();
        SparkMaxConfig rightDriveBConfig = new SparkMaxConfig();
        rightDriveBConfig.follow(m_rightDriveA);
        m_rightDriveA.setInverted(true);

        m_leftEncoder = m_leftDriveA.getEncoder();
        m_rightEncoder = m_rightDriveA.getEncoder();
        m_leftPid = m_leftDriveA.getClosedLoopController();
        m_rightPid = m_rightDriveA.getClosedLoopController();
        m_leftVelocityPidProperty = setupVelocityPidConstants(m_leftDriveA, leftDriveAConfig);
        m_rightVelocityPidProperty = setupVelocityPidConstants(m_rightDriveA, rightDriveAConfig);
        m_leftSmartMotionPidProperty = setupSmartMotionPidConstants(m_leftDriveA, leftDriveAConfig);
        m_rightSmartMotionPidProperty = setupSmartMotionPidConstants(m_rightDriveA, rightDriveAConfig);
        m_leftPositionPidProperty = setupPositionPidConstants(m_leftDriveA, leftDriveAConfig);
        m_rightPositionPidProperty = setupPositionPidConstants(m_rightDriveA, rightDriveAConfig);

        m_gyro = new ADXRS450_Gyro();

        m_differentialDrive = new DifferentialDrive(m_leftDriveA, m_rightDriveA);

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
        m_field = new Field2d();
        SmartDashboard.putData(m_field);

        if (RobotBase.isSimulation()) {
            m_simulator = new DifferentialDrivetrainSimWrapper(
                    DrivetrainConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_leftDriveA, DrivetrainConstants.DRIVE_GEARBOX),
                    new RevMotorControllerSimWrapper(m_rightDriveA, DrivetrainConstants.DRIVE_GEARBOX),
                    RevEncoderSimWrapper.create(m_leftDriveA),
                    RevEncoderSimWrapper.create(m_rightDriveA),
                    new ADXRS450GyroWrapper(m_gyro));
            m_simulator.setRightInverted(false);
        }

        m_leftDriveA.configure(leftDriveAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftDriveB.configure(leftDriveBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightDriveA.configure(rightDriveAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightDriveB.configure(rightDriveBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private PidProperty setupVelocityPidConstants(SparkMax motor, SparkMaxConfig config) {
        return new RevPidPropertyBuilder("Chassis.vel", false, motor, config, PID_SLOT_VELOCITY)
                .addP(0)
                .addFF(0)
                .build();
    }

    private PidProperty setupPositionPidConstants(SparkMax motor, SparkMaxConfig config) {
        return new RevPidPropertyBuilder("Chassis.pos", false, motor, config, PID_SLOT_POSITION)
                .addP(0)
                .addD(0)
                .build();
    }

    private PidProperty setupSmartMotionPidConstants(SparkMax motor, SparkMaxConfig config) {
        return new RevPidPropertyBuilder("Chassis.sm", false, motor, config, PID_SLOT_SMART_MOTION)
                .addP(0)
                .addFF(0)
                .addMaxAcceleration(0)
                .addMaxVelocity(0)
                .build();
    }

    public void setThrottle(double speed) {
        setSpeedAndSteer(speed, 0);
    }

    public void setSpin(double spin) {
        setSpeedAndSteer(0, spin);
    }

    public void setSpeedAndSteer(double speed, double steer) {
        m_differentialDrive.arcadeDrive(speed, steer);
    }

    public void stop() {
        setSpeedAndSteer(0, 0);
    }

    @Override
    public void periodic() {

        m_odometry.update(
                m_gyro.getRotation2d(),
                getLeftDistance(),
                getRightDistance());
        m_field.setRobotPose(m_odometry.getPoseMeters());

        m_leftVelocityPidProperty.updateIfChanged();
        m_rightVelocityPidProperty.updateIfChanged();
        m_leftSmartMotionPidProperty.updateIfChanged();
        m_rightSmartMotionPidProperty.updateIfChanged();
        m_leftPositionPidProperty.updateIfChanged();
        m_rightPositionPidProperty.updateIfChanged();

        SmartDashboard.putNumber("Left Drive", m_leftDriveA.get());
        SmartDashboard.putNumber("Right Drive", m_rightDriveA.get());
        SmartDashboard.putNumber("Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Right Distance", getRightDistance());
        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
    }

    public double getHeading() {
        return m_gyro.getAngle();
    }

    public double getLeftDistance() {
        return m_leftEncoder.getPosition();
    }

    public double getRightDistance() {
        return m_rightEncoder.getPosition();
    }

    public double getLeftVelocity() {
        return m_leftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return m_rightEncoder.getVelocity();
    }

    public double getAverageDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public void driveDistanceCustomControl(double goal) {
        // TODO implement
        m_differentialDrive.feed();
    }

    public void driveDistancePositionControl(double leftDistance, double rightDistance) {
        m_leftPid.setReference(leftDistance, ControlType.kPosition, PID_SLOT_POSITION.value);
        m_rightPid.setReference(rightDistance, ControlType.kPosition, PID_SLOT_POSITION.value);
        m_differentialDrive.feed();

        SmartDashboard.putNumber("Left Position Goal", leftDistance);
        SmartDashboard.putNumber("Right Position Goal", rightDistance);
    }

    public void driveDistanceSmartMotionControl(double leftDistance, double rightDistance) {
        m_leftPid.setReference(leftDistance, ControlType.kMAXMotionPositionControl, PID_SLOT_SMART_MOTION.value);
        m_rightPid.setReference(rightDistance, ControlType.kMAXMotionPositionControl, PID_SLOT_SMART_MOTION.value);
        m_differentialDrive.feed();

        SmartDashboard.putNumber("Left SM Goal", leftDistance);
        SmartDashboard.putNumber("Right SM Goal", rightDistance);
    }

    public void driveWithVelocity(double leftVelocity, double rightVelocity) {
        driveWithVelocity(leftVelocity, rightVelocity, 0, 0);
    }

    public void driveWithVelocity(double leftVelocity, double rightVelocity, double leftAccelMpss, double rightAccelMpss) {

        double staticFrictionLeft = Constants.DrivetrainConstants.KS_VOLTS * Math.signum(leftVelocity);
        double staticFrictionRight = Constants.DrivetrainConstants.KS_VOLTS * Math.signum(rightVelocity);
        double accelerationLeft = Constants.DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER * Math.signum(leftAccelMpss);
        double accelerationRight = Constants.DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER * Math.signum(rightAccelMpss);

        double arbLeft = staticFrictionLeft + accelerationLeft;
        double arbRight = staticFrictionRight + accelerationRight;

        ArbFFUnits arbUnit = ArbFFUnits.kVoltage;

        m_leftPid.setReference(leftVelocity, ControlType.kVelocity, PID_SLOT_VELOCITY.value, arbLeft, arbUnit);
        m_rightPid.setReference(rightVelocity, ControlType.kVelocity, PID_SLOT_VELOCITY.value, arbRight, arbUnit);
        m_differentialDrive.feed();

        SmartDashboard.putNumber("Left Velocity Goal", leftVelocity);
        SmartDashboard.putNumber("Right Velocity Goal", rightVelocity);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
        m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
        m_simulator.resetOdometry(pose);
    }
}
