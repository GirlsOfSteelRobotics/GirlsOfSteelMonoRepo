package com.gos.lib.rev.swerve;


import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.swerve.SwerveModuleSim;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SwerveModuleSimWrapper;


public class RevSwerveModule {
    private final String m_moduleName;

    private final SparkBase m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final RelativeEncoder m_turningRelativeEncoder;
    private final AbsoluteEncoder m_turningAbsoluteEncoder;

    private final SparkClosedLoopController m_drivingPIDController;
    private final PidProperty m_drivingPIDProperty;

    private final SparkClosedLoopController m_turningPIDController;
    private final PidProperty m_turningPIDProperty;

    private final SparkMaxAlerts m_wheelAlerts;
    private final SparkMaxAlerts m_azimuthAlerts;

    private final double m_chassisAngularOffset;

    private SwerveModuleState m_currentState;
    private SwerveModulePosition m_currentPosition;
    private SwerveModuleState m_desiredState;

    private SwerveModuleSimWrapper m_simWrapper;

    private final LoggingUtil m_logger;

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    @SuppressWarnings({"PMD.ExcessiveMethodLength", "PMD.NcssCount"})
    public RevSwerveModule(String moduleName, RevSwerveModuleConstants moduleConstants, int drivingCANId, int azimuthId, double chassisAngularOffset, boolean lockPidConstants) {
        m_moduleName = moduleName;

        switch (moduleConstants.m_driveMotorType) {
        case NEO:
            m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
            break;
        case VORTEX:
            m_drivingSparkMax = new SparkFlex(drivingCANId, MotorType.kBrushless);
            break;
        default:
            throw new IllegalArgumentException();
        }

        m_turningSparkMax = new SparkMax(azimuthId, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        SparkMaxConfig drivingMotorConfig = new SparkMaxConfig();
        SparkMaxConfig turningMotorConfig = new SparkMaxConfig();

        // Request the absolute encoder position / velocity faster than the default period
        turningMotorConfig.signals.absoluteEncoderPositionPeriodMs(10);
        turningMotorConfig.signals.absoluteEncoderVelocityPeriodMs(10);

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningRelativeEncoder = m_turningSparkMax.getEncoder();
        m_turningAbsoluteEncoder = m_turningSparkMax.getAbsoluteEncoder();
        m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();
        drivingMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        turningMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        drivingMotorConfig.encoder.positionConversionFactor(moduleConstants.m_drivingEncoderPositionFactor);
        drivingMotorConfig.encoder.velocityConversionFactor(moduleConstants.m_drivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningMotorConfig.encoder.positionConversionFactor(RevSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningMotorConfig.encoder.velocityConversionFactor(RevSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);
        turningMotorConfig.absoluteEncoder.positionConversionFactor(RevSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningMotorConfig.absoluteEncoder.velocityConversionFactor(RevSwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turningMotorConfig.inverted(RevSwerveModuleConstants.TURNING_ENCODER_INVERTED);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningMotorConfig.closedLoop.positionWrappingEnabled(true);
        turningMotorConfig.closedLoop.positionWrappingMinInput(RevSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
        turningMotorConfig.closedLoop.positionWrappingMaxInput(RevSwerveModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_drivingPIDProperty = new RevPidPropertyBuilder("Swerve Driving PID", lockPidConstants, m_drivingSparkMax, drivingMotorConfig, ClosedLoopSlot.kSlot0)
            .addP(0.04)
            .addD(0)
            .addFF(1 / moduleConstants.m_driveWheelFreeSpeedRps)
            .build();

        m_turningPIDProperty = new RevPidPropertyBuilder("Swerve Turning PID", lockPidConstants, m_turningSparkMax, turningMotorConfig, ClosedLoopSlot.kSlot0)
            .addP(1)
            .addD(0)
            .build();

        m_wheelAlerts = new SparkMaxAlerts(m_drivingSparkMax, "SwerveModuleDrive: " + moduleName);
        m_azimuthAlerts = new SparkMaxAlerts(m_turningSparkMax, "SwerveModuleTurning: " + moduleName);

        drivingMotorConfig.idleMode(RevSwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE);
        turningMotorConfig.idleMode(RevSwerveModuleConstants.TURNING_MOTOR_IDLE_MODE);
        drivingMotorConfig.smartCurrentLimit(RevSwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
        turningMotorConfig.smartCurrentLimit(RevSwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

        if (RobotBase.isSimulation()) {
            DCMotor turningMotor = DCMotor.getNEO(1);
            DCMotor drivingMotor = DCMotor.getNEO(1);
            SwerveModuleSim moduleSim = new SwerveModuleSim(
                turningMotor,
                drivingMotor,
                RevSwerveModuleConstants.WHEEL_DIAMETER_METERS / 2,
                RevSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR,
                moduleConstants.m_drivingMotorReduction,
                1.0,
                1.8, // Seems fishy
                1.1,
                0.8,
                16.0,
                0.001
            );
            m_simWrapper = new SwerveModuleSimWrapper(
                moduleSim,
                new RevMotorControllerSimWrapper(m_drivingSparkMax),
                new RevMotorControllerSimWrapper(m_turningSparkMax),
                RevEncoderSimWrapper.create(m_drivingSparkMax),
                RevEncoderSimWrapper.create(m_turningSparkMax),
                RevSwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI,
                false);
        }

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.configure(drivingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turningSparkMax.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_chassisAngularOffset = RobotBase.isReal() ? chassisAngularOffset : 0;
        m_currentState = new SwerveModuleState();
        m_currentPosition = new SwerveModulePosition();
        m_desiredState = new SwerveModuleState();
        m_desiredState.angle = new Rotation2d(getTurningEncoderAngle());
        m_drivingEncoder.setPosition(0);

        NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("SwerveDrive/" + moduleName);
        m_logger = new LoggingUtil(loggingTable);
        m_logger.addDouble("Goal Angle", () -> m_desiredState.angle.getDegrees());
        m_logger.addDouble("Current Angle", () -> m_currentState.angle.getDegrees());
        m_logger.addDouble("Goal Velocity", () -> m_desiredState.speedMetersPerSecond);
        m_logger.addDouble("Current Velocity", () -> m_currentState.speedMetersPerSecond);
        m_logger.addDouble("Drive Percent Output", m_drivingSparkMax::getAppliedOutput);

        m_logger.addDouble("Abs Encoder", m_turningAbsoluteEncoder::getPosition);
        m_logger.addDouble("Rel Encoder", m_turningRelativeEncoder::getPosition);
    }

    public SwerveModuleSimWrapper getSimWrapper() {
        return m_simWrapper;
    }

    private double getTurningEncoderAngle() {
        if (RobotBase.isReal()) {
            return m_turningAbsoluteEncoder.getPosition();
        }
        else {
            return m_turningRelativeEncoder.getPosition();
        }
    }

    public void periodic() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        m_currentPosition = new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(getTurningEncoderAngle() - m_chassisAngularOffset));
        m_currentState = new SwerveModuleState(m_drivingEncoder.getVelocity(),
            new Rotation2d(getTurningEncoderAngle() - m_chassisAngularOffset));

        m_turningPIDProperty.updateIfChanged();
        m_drivingPIDProperty.updateIfChanged();

        m_logger.updateLogs();
        m_azimuthAlerts.checkAlerts();
        m_wheelAlerts.checkAlerts();
    }

    public SwerveModuleState getDesiredState() {
        return new SwerveModuleState(m_desiredState.speedMetersPerSecond, m_desiredState.angle.minus(new Rotation2d(m_chassisAngularOffset)));
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return m_currentState;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return m_currentPosition;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(getTurningEncoderAngle()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);

        m_desiredState = optimizedDesiredState;
        m_turningPIDController.setReference(m_desiredState.angle.getRadians(), ControlType.kPosition);

    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }

    public void setSpeedPercent(double percentAzimuth, double percentWheel) {
        m_drivingSparkMax.set(percentWheel);
        m_turningSparkMax.set(percentAzimuth);
    }

    public void clearStickyFaults() {
        m_drivingSparkMax.clearFaults();
        m_turningSparkMax.clearFaults();
    }

    public String getName() {
        return m_moduleName;
    }

    public void setCoastMode() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        m_drivingSparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_turningSparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setBrakeMode() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        m_drivingSparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_turningSparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
