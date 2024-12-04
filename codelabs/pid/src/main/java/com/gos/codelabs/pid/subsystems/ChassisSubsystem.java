package com.gos.codelabs.pid.subsystems;

import com.gos.codelabs.pid.Constants;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.SparkClosedLoopController.ArbFFUnits;
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

    private static final int PID_SLOT_VELOCITY = 0;
    private static final int PID_SLOT_SMART_MOTION = 1;
    private static final int PID_SLOT_POSITION = 2;

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
        leftDriveB.restoreFactoryDefaults();
        leftDriveB.follow(m_leftDriveA);

        m_rightDriveA = new SparkMax(Constants.CAN_CHASSIS_RIGHT_A, MotorType.kBrushless);
        SparkMax rightDriveB = new SparkMax(Constants.CAN_CHASSIS_RIGHT_B, MotorType.kBrushless);
        SparkMaxConfig rightDriveAConfig = new SparkMaxConfig();
        rightDriveB.restoreFactoryDefaults();
        rightDriveB.follow(m_rightDriveA);
        m_rightDriveA.setInverted(true);

        m_leftEncoder = m_leftDriveA.getEncoder();
        m_rightEncoder = m_rightDriveA.getEncoder();
        m_leftPid = m_leftDriveA.getClosedLoopController();
        m_rightPid = m_rightDriveA.getClosedLoopController();
        m_leftVelocityPidProperty = setupVelocityPidConstants(m_leftPid);
        m_rightVelocityPidProperty = setupVelocityPidConstants(m_rightPid);
        m_leftSmartMotionPidProperty = setupSmartMotionPidConstants(m_leftPid);
        m_rightSmartMotionPidProperty = setupSmartMotionPidConstants(m_rightPid);
        m_leftPositionPidProperty = setupPositionPidConstants(m_leftPid);
        m_rightPositionPidProperty = setupPositionPidConstants(m_rightPid);

        m_gyro = new ADXRS450_Gyro();

        m_differentialDrive = new DifferentialDrive(m_leftDriveA, m_rightDriveA);

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
        m_field = new Field2d();
        SmartDashboard.putData(m_field);

        if (RobotBase.isSimulation()) {
            m_simulator = new DifferentialDrivetrainSimWrapper(
                    Constants.DrivetrainConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_leftDriveA),
                    new RevMotorControllerSimWrapper(m_rightDriveA),
                    RevEncoderSimWrapper.create(m_leftDriveA),
                    RevEncoderSimWrapper.create(m_rightDriveA),
                    new ADXRS450GyroWrapper(m_gyro));
            m_simulator.setRightInverted(false);
        }

        m_leftDriveA.configure(leftDriveAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftDriveB.burnFlash();
        m_rightDriveA.configure(rightDriveAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightDriveB.burnFlash();
    }

    private PidProperty setupVelocityPidConstants(SparkClosedLoopController pidController) {
        return new RevPidPropertyBuilder("Chassis.vel", false, pidController, PID_SLOT_VELOCITY)
                .addP(0)
                .addFF(0)
                .build();
    }

    private PidProperty setupPositionPidConstants(SparkClosedLoopController pidController) {
        return new RevPidPropertyBuilder("Chassis.pos", false, pidController, PID_SLOT_POSITION)
                .addP(0)
                .addD(0)
                .build();
    }

    private PidProperty setupSmartMotionPidConstants(SparkClosedLoopController pidController) {
        return new RevPidPropertyBuilder("Chassis.sm", false, pidController, PID_SLOT_SMART_MOTION)
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
        m_leftPid.setReference(leftDistance, ControlType.kPosition, PID_SLOT_POSITION);
        m_rightPid.setReference(rightDistance, ControlType.kPosition, PID_SLOT_POSITION);
        m_differentialDrive.feed();

        SmartDashboard.putNumber("Left Position Goal", leftDistance);
        SmartDashboard.putNumber("Right Position Goal", rightDistance);
    }

    public void driveDistanceSmartMotionControl(double leftDistance, double rightDistance) {
        m_leftPid.setReference(leftDistance, ControlType.kSmartMotion, PID_SLOT_SMART_MOTION);
        m_rightPid.setReference(rightDistance, ControlType.kSmartMotion, PID_SLOT_SMART_MOTION);
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

        m_leftPid.setReference(leftVelocity, ControlType.kVelocity, PID_SLOT_VELOCITY, arbLeft, arbUnit);
        m_rightPid.setReference(rightVelocity, ControlType.kVelocity, PID_SLOT_VELOCITY, arbRight, arbUnit);
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
