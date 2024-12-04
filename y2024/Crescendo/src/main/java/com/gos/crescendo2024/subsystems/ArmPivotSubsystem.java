package com.gos.crescendo2024.subsystems;

import com.gos.crescendo2024.Constants;
import com.gos.crescendo2024.FieldConstants;
import com.gos.crescendo2024.SpeakerLookupTable;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.feedforward.ArmFeedForwardProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiProfiledPidPropertyBuilder;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SingleJointedArmSimWrapper;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@SuppressWarnings("PMD.GodClass")
public class ArmPivotSubsystem extends SubsystemBase {
    private static final GosDoubleProperty ARM_INTAKE_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "intakeAngle", 356);
    public static final GosDoubleProperty ARM_TUNABLE_SPEAKER_ANGLE = new GosDoubleProperty(false, "tunableSpeakerAngle", 0);
    public static final GosDoubleProperty MIDDLE_SUBWOOFER_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "middleSpeakerScoreAngle", 11);
    public static final GosDoubleProperty SIDE_SUBWOOFER_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "sideSpeakerScoreAngle", 14); //chnanged middle and side to 11 and 14 from 13 and 16 because of practice field issues
    private static final GosDoubleProperty ARM_AMP_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "ampScoreAngle", 90);
    public static final GosDoubleProperty ARM_FEEDER_ANGLE = new GosDoubleProperty(false, "Feeding: allianceFeederAngle", 32);
    public static final GosDoubleProperty ARM_PREP_HANGER_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "prepHangerAngle", 95);
    public static final GosDoubleProperty ARM_MAX_ANGLE_FOR_CHAIN = new GosDoubleProperty(false, "Arm: MaxAngleForChain", 20);

    private static final double ARM_MAX_ANGLE = 102; //from hanger testing day 3/5/24

    public static final GosDoubleProperty SPIKE_TOP_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "arm spike top angle", 40);
    public static final GosDoubleProperty SPIKE_MIDDLE_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "arm spike middle angle", 32);
    public static final GosDoubleProperty SPIKE_BOTTOM_ANGLE = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "arm spike bottom angle", 32);


    private static final double GEAR_RATIO;
    private static final boolean USE_ABSOLUTE_ENCODER = false;

    private static final double ALLOWABLE_ERROR = .5; //TODO change allowable error to make it more accurate or to make scoring faster

    static {
        if (Constants.IS_COMPETITION_ROBOT) {
            GEAR_RATIO = (58.0 / 15.0) * 45; //5::5 to 9::5
        } else {
            GEAR_RATIO = (58.0 / 12.0) * (3.0) * (5.0);
        }
    }

    private final SparkMax m_pivotMotor;
    private final SparkMax m_followMotor;
    private final RelativeEncoder m_pivotMotorEncoder;
    private final AbsoluteEncoder m_pivotAbsEncoder;
    private final LoggingUtil m_networkTableEntriesPivot;
    private final SparkMaxAlerts m_armPivotMotorErrorAlerts;
    private final SparkMaxAlerts m_armPivotFollowerErrorAlerts;

    private final SparkClosedLoopController m_sparkPidController;
    private final PidProperty m_sparkPidProperties;
    private final ProfiledPIDController m_profilePID;
    private final PidProperty m_profilePidProperties;
    private final ArmFeedForwardProperty m_wpiFeedForward;
    private double m_armGoalAngle = Double.MIN_VALUE;
    private SingleJointedArmSimWrapper m_pivotSimulator;

    private final SpeakerLookupTable m_speakerTable;

    public ArmPivotSubsystem() {
        m_pivotMotor = new SparkMax(Constants.ARM_PIVOT, MotorType.kBrushless);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(60);
        m_pivotMotor.setInverted(false);

        m_followMotor = new SparkMax(Constants.ARM_PIVOT_FOLLOW, MotorType.kBrushless);
        SparkMaxConfig followMotorConfig = new SparkMaxConfig();
        followMotorConfig.follow(m_pivotMotor, true);
        followMotorConfig.idleMode(IdleMode.kBrake);
        followMotorConfig.smartCurrentLimit(60);

        // Request the absolute encoder position / velocity faster than the default period
        pivotMotorConfig.signals.absoluteEncoderPositionPeriodMs(20);
        pivotMotorConfig.signals.absoluteEncoderVelocityPeriodMs(20);

        m_pivotMotorEncoder = m_pivotMotor.getEncoder();
        pivotMotorConfig.encoder.positionConversionFactor(360.0 / GEAR_RATIO);
        pivotMotorConfig.encoder.velocityConversionFactor(360.0 / GEAR_RATIO / 60);

        m_pivotAbsEncoder = m_pivotMotor.getAbsoluteEncoder();
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(360.0);
        pivotMotorConfig.absoluteEncoder.velocityConversionFactor(360.0 / 60);
        pivotMotorConfig.absoluteEncoder.inverted(false);
        pivotMotorConfig.absoluteEncoder.zeroOffset(277.11 + 2);

        m_speakerTable = new SpeakerLookupTable();

        m_sparkPidController = m_pivotMotor.getClosedLoopController();
        if (USE_ABSOLUTE_ENCODER) {
            pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        } else {
            pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        }
        pivotMotorConfig.closedLoop.positionWrappingEnabled(true);
        pivotMotorConfig.closedLoop.positionWrappingMinInput(0);
        pivotMotorConfig.closedLoop.positionWrappingMaxInput(360);
        m_sparkPidProperties = new RevPidPropertyBuilder("Arm Pivot", Constants.DEFAULT_CONSTANT_PROPERTIES, m_sparkPidController, 0)
            .addP(0.18)
            .addI(0)
            .addD(0)
            .build();
        m_profilePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        m_profilePID.enableContinuousInput(0, 360);
        m_profilePidProperties = new WpiProfiledPidPropertyBuilder("Arm Profile PID", false, m_profilePID)
            .addMaxVelocity(200)
            .addMaxAcceleration(300)
            .build();
        m_wpiFeedForward = new ArmFeedForwardProperty("Arm Pivot Profile ff", Constants.DEFAULT_CONSTANT_PROPERTIES)
            .addKs(0)
            .addKff(2.2)
            .addKg(1.2);

        m_networkTableEntriesPivot = new LoggingUtil("Arm Pivot Subsystem");
        m_networkTableEntriesPivot.addDouble("Output", m_pivotMotor::getAppliedOutput);
        m_networkTableEntriesPivot.addDouble("Arm Current", m_pivotMotor::getOutputCurrent);
        m_networkTableEntriesPivot.addDouble("Arm Speed", m_pivotMotor::get);
        m_networkTableEntriesPivot.addDouble("Abs Encoder Position", m_pivotAbsEncoder::getPosition);
        m_networkTableEntriesPivot.addDouble("Abs Encoder Velocity", m_pivotAbsEncoder::getVelocity);
        m_networkTableEntriesPivot.addDouble("Rel Encoder Position", () -> m_pivotMotorEncoder.getPosition() % 360);
        m_networkTableEntriesPivot.addDouble("Rel Encoder Velocity", m_pivotMotorEncoder::getVelocity);
        m_networkTableEntriesPivot.addBoolean("Arm At Goal", this::isArmAtGoal);
        m_networkTableEntriesPivot.addDouble("Setpoint Position", () -> m_profilePID.getSetpoint().position);
        m_networkTableEntriesPivot.addDouble("Setpoint Velocity", () -> m_profilePID.getSetpoint().velocity);

        m_armPivotMotorErrorAlerts = new SparkMaxAlerts(m_pivotMotor, "arm pivot motor");
        m_armPivotFollowerErrorAlerts = new SparkMaxAlerts(m_followMotor, "arm follower");

        m_pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNeo550(1);
            SingleJointedArmSim armSim = new SingleJointedArmSim(gearbox, 252, 1,
                0.381, Units.degreesToRadians(-2), Units.degreesToRadians(90), true, 0);
            m_pivotSimulator = new SingleJointedArmSimWrapper(armSim, new RevMotorControllerSimWrapper(m_pivotMotor),
                RevEncoderSimWrapper.create(m_pivotMotor), true);
        }

        syncRelativeEncoder();
    }

    public void clearStickyFaults() {
        m_pivotMotor.clearFaults();
        m_followMotor.clearFaults();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            syncRelativeEncoder();
        }
        m_networkTableEntriesPivot.updateLogs();
        m_armPivotMotorErrorAlerts.checkAlerts();
        m_armPivotFollowerErrorAlerts.checkAlerts();

        m_sparkPidProperties.updateIfChanged();
        m_wpiFeedForward.updateIfChanged();
        m_profilePidProperties.updateIfChanged();
    }

    public void moveArmToAngle(double goalAngle) {
        if (Math.abs(m_armGoalAngle - goalAngle) > 2) {
            resetPidController();
        }

        m_armGoalAngle = goalAngle;
        double currentAngle = getAngle();
        if (currentAngle < ARM_MAX_ANGLE || MathUtil.inputModulus(goalAngle, -180, 180) < MathUtil.inputModulus(currentAngle, -180, 180)) {
            m_profilePID.calculate(currentAngle, goalAngle);
            TrapezoidProfile.State setpoint = m_profilePID.getSetpoint();
            double feedForwardVolts = m_wpiFeedForward.calculate(
                Units.degreesToRadians(currentAngle),
                Units.degreesToRadians(setpoint.velocity));


            m_sparkPidController.setReference(setpoint.position, ControlType.kPosition, 0, feedForwardVolts);
            SmartDashboard.putNumber("feedForwardVolts", feedForwardVolts);
        }
        else {
            stopArmMotor();
        }
    }

    private void resetPidController() {
        m_profilePID.reset(getAngle(), getEncoderVel());
    }


    public void pivotUsingSpeakerLookupTable(Supplier<Pose2d> roboMan) {
        moveArmToAngle(getPivotAngleUsingSpeakerLookupTable(roboMan));
    }

    public double getPivotAngleUsingSpeakerLookupTable(Supplier<Pose2d> roboMan) {
        Pose2d speaker = FieldConstants.Speaker.CENTER_SPEAKER_OPENING.getPose();
        Translation2d roboManTranslation = roboMan.get().getTranslation();
        double distanceToSpeaker = roboManTranslation.getDistance(speaker.getTranslation());
        return m_speakerTable.getAngleTable(distanceToSpeaker);
    }

    @Override
    public void simulationPeriodic() {
        m_pivotSimulator.update();
    }

    public void stopArmMotor() {
        m_pivotMotor.set(0);
        m_armGoalAngle = Double.MIN_VALUE;
    }

    public double getAngle() {
        if (RobotBase.isReal() && USE_ABSOLUTE_ENCODER) {
            return getAbsoluteEncoderAngle();
        }
        else {
            return getRelativeEncoderAngle();
        }
    }

    public double getAbsoluteEncoderAngle() {
        double angle = m_pivotAbsEncoder.getPosition();
        angle = MathUtil.inputModulus(angle, -180, 180);
        return angle;
    }

    public double getRelativeEncoderAngle() {
        double angle = m_pivotMotorEncoder.getPosition();
        angle = MathUtil.inputModulus(angle, -180, 180);
        return angle;
    }

    public double getEncoderVel() {
        return m_pivotMotorEncoder.getVelocity();
    }

    public void setArmMotorSpeed(double speed) {
        if (getAngle() < ARM_MAX_ANGLE || speed < 0) {
            m_pivotMotor.set(speed);
        }
        else {
            m_sparkPidController.setReference(ARM_MAX_ANGLE, ControlType.kPosition);
        }
    }

    public void setVoltage(double outputVolts) {
        m_pivotMotor.setVoltage(outputVolts);
    }

    public double getArmAngleGoal() {
        return m_armGoalAngle;
    }


    public double getPivotMotorPercentage() {
        return m_pivotMotor.getAppliedOutput();
    }

    public double getVoltage() {
        if (RobotBase.isReal()) {
            return m_pivotMotor.getBusVoltage();
        }
        return m_pivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    }

    public boolean isArmAtGoal() {
        double error = m_armGoalAngle - getAngle();
        return Math.abs(error) < ALLOWABLE_ERROR;
    }

    private void syncRelativeEncoder() {
        m_pivotMotorEncoder.setPosition(m_pivotAbsEncoder.getPosition());
    }

    public boolean areArmEncodersGood() {
        return Math.abs(getRelativeEncoderAngle() - getAbsoluteEncoderAngle()) <= ALLOWABLE_ERROR;
    }

    public boolean canGoUnderChain() {
        return getAngle() < ARM_MAX_ANGLE_FOR_CHAIN.getValue();
    }

    public void setIdleMode(IdleMode idleMode) {
        pivotMotorConfig.idleMode(idleMode);
        followMotorConfig.idleMode(idleMode);
    }

    /////////////////////////////////////
    // Command Factories
    /////////////////////////////////////
    private Command createResetPidControllerCommand() {
        return runOnce(this::resetPidController);
    }

    public Command createPivotUsingSpeakerTableCommand(Supplier<Pose2d> roboMan) {
        return this.runEnd(() -> this.pivotUsingSpeakerLookupTable(roboMan), this::stopArmMotor).withName("pivot from robot pose");
    }

    public Command createMoveArmToAngleCommand(DoubleSupplier angleSupplier) {
        return createResetPidControllerCommand().andThen(
            runEnd(() -> moveArmToAngle(angleSupplier.getAsDouble()), this::stopArmMotor));
    }

    public Command createMoveArmToAngleCommand(double goalAngle) {
        return createMoveArmToAngleCommand(() -> goalAngle).withName("arm to " + goalAngle);
    }

    public Command createMoveArmToGroundIntakeAngleCommand() {
        return createMoveArmToAngleCommand(ARM_INTAKE_ANGLE::getValue).withName("arm to ground intake angle");
    }

    public Command createMoveArmToAmpAngleCommand() {
        return createMoveArmToAngleCommand(ARM_AMP_ANGLE::getValue).withName("arm to amp angle");
    }

    public Command createMoveArmToMiddleSpeakerAngleCommand() {
        return createMoveArmToAngleCommand(MIDDLE_SUBWOOFER_ANGLE::getValue).withName("arm to middle speaker angle");
    }

    public Command createMoveArmToSideSpeakerAngleCommand() {
        return createMoveArmToAngleCommand(SIDE_SUBWOOFER_ANGLE::getValue).withName("arm to side speaker angle");
    }

    public Command createMoveArmFeederAngleCommand() {
        return createMoveArmToAngleCommand(ARM_FEEDER_ANGLE::getValue).withName("move arm to feeder angle");
    }

    public Command createSyncRelativeEncoderCommand() {
        return run(this::syncRelativeEncoder).ignoringDisable(true).withName("arm: sync encoder");
    }

    public Command createMoveArmToTunableSpeakerAngleCommand() {
        return createMoveArmToAngleCommand(ARM_TUNABLE_SPEAKER_ANGLE::getValue).withName("arm to tunable speaker angle");
    }

    public Command createMoveArmToPrepHangerAngleCommand() {
        return createMoveArmToAngleCommand(ARM_PREP_HANGER_ANGLE::getValue)
            .until(this::isArmAtGoal).withName("arm to prep hanger angle");
    }

    public Command createPivotToCoastModeCommand() {
        return this.runEnd(
                () -> setIdleMode(IdleMode.kCoast),
                () -> setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true).withName("Pivot to Coast");
    }
}
