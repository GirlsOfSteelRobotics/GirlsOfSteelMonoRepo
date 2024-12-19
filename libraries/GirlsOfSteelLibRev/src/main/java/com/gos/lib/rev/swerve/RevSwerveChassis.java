package com.gos.lib.rev.swerve;

import com.gos.lib.rev.swerve.config.RevSwerveChassisConstants;
import com.gos.lib.rev.swerve.config.RevSwerveModuleConstants;
import com.gos.lib.swerve.SwerveDrivePublisher;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.snobotv2.module_wrappers.BaseGyroWrapper;
import org.snobotv2.sim_wrappers.SwerveModuleSimWrapper;
import org.snobotv2.sim_wrappers.SwerveSimWrapper;

import java.util.List;
import java.util.function.Supplier;

public class RevSwerveChassis implements AutoCloseable {
    private final double m_maxSpeedMetersPerSecond;
    private final double m_maxAngularSpeed;

    private final SwerveDriveKinematics m_kinematics;

    // Create MAXSwerveModules
    private final RevSwerveModule m_frontLeft;
    private final RevSwerveModule m_frontRight;
    private final RevSwerveModule m_backLeft;
    private final RevSwerveModule m_backRight;

    private final RevSwerveModule[] m_modules;

    // The gyro sensor
    private final Supplier<Rotation2d> m_gyroAngleSupplier;

    // Odometry class for tracking robot pose
    private final SwerveDriveOdometry m_odometry;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // Publish Telemetry
    private final SwerveDrivePublisher m_swervePublisher;

    // Simulation
    private SwerveSimWrapper m_simulator;

    /** Creates a new DriveSubsystem. */
    public RevSwerveChassis(RevSwerveChassisConstants chassisConstants, Supplier<Rotation2d> gyroAngleSupplier, BaseGyroWrapper gyroSimulator) {
        RevSwerveModuleConstants moduleConstants = chassisConstants.getModuleConstants();

        m_frontLeft = new RevSwerveModule(
            "FL",
            moduleConstants,
            chassisConstants.m_frontLeftDrivingCanId,
            chassisConstants.m_frontLeftTurningCanId,
            RevSwerveChassisConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
            chassisConstants.m_drivingClosedLoopParameters,
            chassisConstants.m_turningClosedLoopParameters,
            chassisConstants.m_lockPidConstants);
        m_frontRight = new RevSwerveModule(
            "FR",
            moduleConstants,
            chassisConstants.m_frontRightDrivingCanId,
            chassisConstants.m_frontRightTurningCanId,
            RevSwerveChassisConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
            chassisConstants.m_drivingClosedLoopParameters,
            chassisConstants.m_turningClosedLoopParameters,
            chassisConstants.m_lockPidConstants);
        m_backLeft = new RevSwerveModule(
            "BL",
            moduleConstants,
            chassisConstants.m_rearLeftDrivingCanId,
            chassisConstants.m_rearLeftTurningCanId,
            RevSwerveChassisConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
            chassisConstants.m_drivingClosedLoopParameters,
            chassisConstants.m_turningClosedLoopParameters,
            chassisConstants.m_lockPidConstants);
        m_backRight = new RevSwerveModule(
            "BR",
            moduleConstants,
            chassisConstants.m_rearRightDrivingCanId,
            chassisConstants.m_rearRightTurningCanId,
            RevSwerveChassisConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
            chassisConstants.m_drivingClosedLoopParameters,
            chassisConstants.m_turningClosedLoopParameters,
            chassisConstants.m_lockPidConstants);
        m_modules = new RevSwerveModule[]{m_frontLeft, m_frontRight, m_backLeft, m_backRight};

        m_kinematics = new SwerveDriveKinematics(
            new Translation2d(chassisConstants.m_wheelBase / 2, chassisConstants.m_trackWidth / 2),
            new Translation2d(chassisConstants.m_wheelBase / 2, -chassisConstants.m_trackWidth / 2),
            new Translation2d(-chassisConstants.m_wheelBase / 2, chassisConstants.m_trackWidth / 2),
            new Translation2d(-chassisConstants.m_wheelBase / 2, -chassisConstants.m_trackWidth / 2)
        );

        m_gyroAngleSupplier = gyroAngleSupplier;

        m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyroAngleSupplier.get(),
            getModulePositions());
        m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyroAngleSupplier.get(), getModulePositions(), new Pose2d());

        m_swervePublisher = new SwerveDrivePublisher();

        m_maxSpeedMetersPerSecond = chassisConstants.m_maxSpeedMetersPerSecond;
        m_maxAngularSpeed = chassisConstants.m_maxAngularSpeed;

        if (RobotBase.isSimulation()) {
            List<SwerveModuleSimWrapper> moduleSims = List.of(
                m_frontLeft.getSimWrapper(),
                m_frontRight.getSimWrapper(),
                m_backLeft.getSimWrapper(),
                m_backRight.getSimWrapper());
            m_simulator = new SwerveSimWrapper(chassisConstants.m_wheelBase, chassisConstants.m_trackWidth, 64.0, 1.0, moduleSims, gyroSimulator);
        }
    }

    @Override
    public void close() {
        for (RevSwerveModule module : m_modules) {
            module.close();
        }
    }

    public void periodic() {
        for (RevSwerveModule module : m_modules) {
            module.periodic();
        }

        SwerveModulePosition[] modulePositions = getModulePositions();
        Rotation2d rotation = m_gyroAngleSupplier.get();
        m_odometry.update(rotation, modulePositions);
        m_poseEstimator.update(rotation, modulePositions);

        SwerveModuleState[] moduleStates = getModuleStates();
        SwerveModuleState[] desiredStates = getModuleDesiredStates();
        m_swervePublisher.setMeasuredStates(moduleStates);
        m_swervePublisher.setDesiredStates(desiredStates);
        m_swervePublisher.setRobotRotation(getEstimatedPosition().getRotation());
    }

    public void updateSimulator() {
        m_simulator.update();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getEstimatedPosition() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> estimatedStdDev) {
        m_poseEstimator.addVisionMeasurement(pose, timestampSeconds, estimatedStdDev);
    }

    public Pose2d getOdometryPosition() {
        return m_odometry.getPoseMeters();
    }

    public void syncOdometryWithPoseEstimator() {
        m_odometry.resetPosition(m_gyroAngleSupplier.get(), getModulePositions(), m_poseEstimator.getEstimatedPosition());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyroAngleSupplier.get(), getModulePositions(), pose);
        m_poseEstimator.resetPosition(m_gyroAngleSupplier.get(), getModulePositions(), pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Joystick speed of the robot in the x direction (forward).
     * @param ySpeed        Joystick speed of the robot in the y direction (sideways).
     * @param rot           Joystick angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void driveWithJoysticks(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * m_maxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * m_maxSpeedMetersPerSecond;
        double rotDelivered = rot * m_maxAngularSpeed;

        ChassisSpeeds desiredSpeed;
        if (fieldRelative) {
            desiredSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getEstimatedPosition().getRotation());
        } else {
            desiredSpeed = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        }
        setChassisSpeeds(desiredSpeed);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setWheelsToXShape() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState... desiredStates) {
        for (int i = 0; i < 4; i++) {
            m_modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public final SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i += 1) {
            modulePositions[i] = m_modules[i].getPosition();
        }
        return modulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] modulePositions = new SwerveModuleState[4];
        for (int i = 0; i < 4; i += 1) {
            modulePositions[i] = m_modules[i].getState();
        }
        return modulePositions;
    }

    private SwerveModuleState[] getModuleDesiredStates() {
        SwerveModuleState[] modulePositions = new SwerveModuleState[4];
        for (int i = 0; i < 4; i += 1) {
            modulePositions[i] = m_modules[i].getDesiredState();
        }
        return modulePositions;
    }

    public void stop() {
        for (int i = 0; i < 4; i += 1) {
            m_modules[i].setSpeedPercent(0, 0);
        }
    }

    public void setChassisSpeedsPercent(double wheelPercent, double azimuthPercent) {
        for (RevSwerveModule module: m_modules) {
            module.setSpeedPercent(azimuthPercent, wheelPercent);
        }
    }

    public void setModuleState(int moduleId, double degrees, double velocity) {
        m_modules[moduleId].setDesiredState(new SwerveModuleState(velocity, Rotation2d.fromDegrees(degrees)));
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, m_maxSpeedMetersPerSecond);
        setModuleStates(moduleStates);
    }

    public ChassisSpeeds getChassisSpeed() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public void clearStickyFaults() {
        for (RevSwerveModule module: m_modules) {
            module.clearStickyFaults();
        }
    }

    public String getModuleName(int moduleId) {
        return m_modules[moduleId].getName();
    }

    public void setModuleBrakeMode() {
        for (RevSwerveModule module: m_modules) {
            module.setBrakeMode();
        }
    }

    private void setModuleCoastMode() {
        for (RevSwerveModule module: m_modules) {
            module.setCoastMode();
        }
    }

    public void setModulesToPushMode(double wheelAngleDeg) {
        setModuleCoastMode();

        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(wheelAngleDeg));
        for (RevSwerveModule module: m_modules) {
            module.setDesiredState(state);
        }
    }


    public SparkBaseConfigAccessor getTurningMotorConfig(int moduleId) {
        return m_modules[moduleId].getTurningMotorConfig();
    }

    public SparkBaseConfigAccessor getDrivingMotorConfig(int moduleId) {
        return m_modules[moduleId].getDrivingMotorConfig();
    }

    public void characterizeDriveMotors(Voltage volts) {
        for (RevSwerveModule module: m_modules) {
            module.characterizeDriveMotor(volts);
        }
    }

    public void characterizeSteerMotors(Voltage volts) {
        for (RevSwerveModule module: m_modules) {
            module.characterizeSteerMotor(volts);
        }
    }
}
