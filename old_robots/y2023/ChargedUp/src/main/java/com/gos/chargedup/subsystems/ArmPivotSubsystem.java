package com.gos.chargedup.subsystems;

import com.gos.chargedup.AutoAimNodePositions;
import com.gos.chargedup.AutoPivotHeight;
import com.gos.chargedup.Constants;
import com.gos.chargedup.GamePieceType;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.feedforward.ArmFeedForwardProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiProfiledPidPropertyBuilder;
import com.gos.lib.rev.SparkMaxUtil;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.gos.lib.rev.checklists.SparkMaxMotorsMoveChecklist;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SingleJointedArmSimWrapper;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


@SuppressWarnings({"PMD.GodClass", "PMD.ExcessivePublicCount"})
public class ArmPivotSubsystem extends SubsystemBase {

    private static final GosDoubleProperty ALLOWABLE_ERROR = new GosDoubleProperty(false, "Pivot Arm Allowable Error", 1.5);
    private static final GosDoubleProperty ALLOWABLE_VELOCITY_ERROR = new GosDoubleProperty(Constants.DEFAULT_LOCK_PROPERTIES, "Pivot Arm Allowable Velocity Error", 20);

    private static final double ARM_MOTOR_SPEED = 0.20;
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(15);

    private static final double GEAR_RATIO = 45.0 * 4.0;

    // Arm Setpoints
    public static final double HUMAN_PLAYER_ANGLE;
    public static final double ARM_CUBE_MIDDLE_DEG;
    public static final double ARM_CUBE_HIGH_DEG;
    public static final double ARM_CONE_MIDDLE_DEG;
    public static final double ARM_CONE_HIGH_DEG;
    public static final double ARM_SCORE_LOW_DEG;
    public static final double GROUND_PICKUP_ANGLE;
    public static final double HOME_ANGLE;
    public static final double MIN_ANGLE_DEG;
    public static final double MAX_ANGLE_DEG;

    static {
        //toDo: make these values work as expected
        if (Constants.IS_ROBOT_BLOSSOM) {
            HUMAN_PLAYER_ANGLE = 18;
            ARM_CUBE_MIDDLE_DEG = 0;
            ARM_CUBE_HIGH_DEG = 23;
            ARM_CONE_MIDDLE_DEG = 14;
            ARM_CONE_HIGH_DEG = 25;
            MIN_ANGLE_DEG = -60;
            MAX_ANGLE_DEG = 50;
            HOME_ANGLE = MIN_ANGLE_DEG + 5;
            GROUND_PICKUP_ANGLE = -38;
            ARM_SCORE_LOW_DEG = -35;

        } else {
            HUMAN_PLAYER_ANGLE = 10;
            ARM_CUBE_MIDDLE_DEG = 0;
            ARM_CUBE_HIGH_DEG = 12;
            ARM_CONE_MIDDLE_DEG = 15;
            ARM_CONE_HIGH_DEG = 30;
            MIN_ANGLE_DEG = -60;
            MAX_ANGLE_DEG = 50;
            HOME_ANGLE = -45;
            GROUND_PICKUP_ANGLE = -20;
            ARM_SCORE_LOW_DEG = -25;
        }
    }

    public static final double ARM_HIT_INTAKE_ANGLE = 15;

    // Sim Stuff
    private static final double GEARING =  252.0;
    private static final double J_KG_METERS_SQUARED = 1;
    private static final double MIN_ANGLE_RADS = Math.toRadians(MIN_ANGLE_DEG);
    private static final double MAX_ANGLE_RADS = Math.toRadians(MAX_ANGLE_DEG);
    private static final boolean SIMULATE_GRAVITY = true;

    private final SparkMax m_pivotMotor;
    private final RelativeEncoder m_pivotMotorEncoder;
    private final SparkClosedLoopController m_sparkPidController;
    private final PidProperty m_sparkPidProperties;

    private final ProfiledPIDController m_wpiPidController;
    private final PidProperty m_wpiPidProperties;
    private final ArmFeedForwardProperty m_wpiFeedForward;

    private final SparkAbsoluteEncoder m_absoluteEncoder;

    private final DigitalInput m_lowerLimitSwitch;
    private final DigitalInput m_upperLimitSwitch;

    private double m_armAngleGoal = Double.MIN_VALUE;

    private final LoggingUtil m_networkTableEntries;

    private final NetworkTableEntry m_profilePositionGoalEntry;
    private final NetworkTableEntry m_profileVelocityGoalEntry;
    private final NetworkTableEntry m_pidArbitraryFeedForwardEntry;

    private final SparkMaxAlerts m_pivotErrorAlert;

    private SingleJointedArmSimWrapper m_pivotSimulator;

    public ArmPivotSubsystem() {
        m_pivotMotor = new SparkMax(Constants.PIVOT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        m_pivotMotor.setInverted(true);
        pivotMotorConfig.smartCurrentLimit(60);
        m_pivotMotorEncoder = m_pivotMotor.getEncoder();

        m_absoluteEncoder = m_pivotMotor.getAbsoluteEncoder();
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(360.0);
        pivotMotorConfig.absoluteEncoder.velocityConversionFactor(360.0 / 60);
        pivotMotorConfig.absoluteEncoder.inverted(true);
        pivotMotorConfig.absoluteEncoder.zeroOffset(21.5);

        m_sparkPidController = m_pivotMotor.getClosedLoopController();
        if (Constants.IS_ROBOT_BLOSSOM) {
            pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            pivotMotorConfig.closedLoop.positionWrappingEnabled(true);
            pivotMotorConfig.closedLoop.positionWrappingMinInput(0);
            pivotMotorConfig.closedLoop.positionWrappingMaxInput(360);
        } else {
            pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        }

        pivotMotorConfig.encoder.positionConversionFactor(360.0 / GEAR_RATIO);
        pivotMotorConfig.encoder.velocityConversionFactor(360.0 / GEAR_RATIO / 60.0);

        m_lowerLimitSwitch = new DigitalInput(Constants.INTAKE_LOWER_LIMIT_SWITCH);
        m_upperLimitSwitch = new DigitalInput(Constants.INTAKE_UPPER_LIMIT_SWITCH);
        m_sparkPidProperties = setupSparkPidValues(m_sparkPidController);

        m_wpiPidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        m_wpiPidProperties = new WpiProfiledPidPropertyBuilder("Pivot Arm Motion Profile", false, m_wpiPidController)
            .addP(0.0073)
            .addMaxAcceleration(80)
            .addMaxVelocity(80)
            .build();
        m_wpiFeedForward = new ArmFeedForwardProperty("Pivot Arm Motion Profile", false)
            .addKff(3.455)
            .addKs(0.10072)
            .addKg(0.22);

        m_networkTableEntries = new LoggingUtil("Arm Subsystem");
        m_networkTableEntries.addBoolean("Arm Lower LS", this::isLowerLimitSwitchedPressed);
        m_networkTableEntries.addBoolean("Arm Upper LS", this::isUpperLimitSwitchedPressed);
        m_networkTableEntries.addDouble("Arm Encoder (deg)", this::getArmAngleDeg2);
        m_networkTableEntries.addDouble("Absolute Encoder Entry", this::getAbsoluteEncoderAngle2);
        m_networkTableEntries.addDouble("Encoder Drift", this::getAbsoluteEncoderBuiltInDrift);
        m_networkTableEntries.addDouble("Arm Velocity", this::getArmVelocityDegPerSec2);
        m_networkTableEntries.addDouble("Arm Goal (deg)", () -> m_armAngleGoal);

        NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Arm Subsystem");
        m_profilePositionGoalEntry = loggingTable.getEntry("Profile Angle Goal");
        m_profileVelocityGoalEntry = loggingTable.getEntry("Velocity Goal");
        m_pidArbitraryFeedForwardEntry = loggingTable.getEntry("Arbitrary FF");
        m_pivotErrorAlert = new SparkMaxAlerts(m_pivotMotor, "arm pivot motor ");

        if (RobotBase.isSimulation()) {
            SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNeo550(1), GEARING, J_KG_METERS_SQUARED,
                ARM_LENGTH_METERS, MIN_ANGLE_RADS, MAX_ANGLE_RADS, SIMULATE_GRAVITY, 0);
            m_pivotSimulator = new SingleJointedArmSimWrapper(armSim, new RevMotorControllerSimWrapper(m_pivotMotor),
                RevEncoderSimWrapper.create(m_pivotMotor), true);
        }

        if (Constants.IS_ROBOT_BLOSSOM) {
            syncMotorEncoderToAbsoluteEncoder();
        }
        else {
            resetPivotEncoder(MIN_ANGLE_DEG);
        }

        m_pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public final void syncMotorEncoderToAbsoluteEncoder() {
        resetPivotEncoder(getAbsoluteEncoderAngle2());
    }

    private boolean useAbsoluteEncoder() {
        return Constants.IS_ROBOT_BLOSSOM && !RobotBase.isSimulation();
    }

    public double getFeedbackAngleDeg() {
        if (useAbsoluteEncoder()) {
            return getAbsoluteEncoderAngle2();
        } else {
            return getArmAngleDeg2();
        }
    }

    public double getFeedbackVelocityDegPerSec() {
        if (useAbsoluteEncoder()) {
            return getAbsoluteEncoderVelocity2();
        } else {
            return getArmVelocityDegPerSec2();
        }
    }

    public final double getAbsoluteEncoderAngle2() {
        double val = m_absoluteEncoder.getPosition();
        if (val > 180) {
            val -= 360;
        }
        if (val < -180) {
            val += 360;
        }

        return val;
    }

    public double getAbsoluteEncoderBuiltInDrift() {
        return getAbsoluteEncoderAngle2() - getArmAngleDeg2();
    }

    private double getAbsoluteEncoderVelocity2() {
        return m_absoluteEncoder.getVelocity();
    }

    private PidProperty setupSparkPidValues(SparkClosedLoopController pidController) {
        ///
        // Full retract:
        // kp=0.000400
        // kf=0.005000
        // kd=0.005000
        if (Constants.IS_ROBOT_BLOSSOM) {
            return new RevPidPropertyBuilder("Pivot Arm", Constants.DEFAULT_LOCK_PROPERTIES, pidController, 0)
                .addP(0.03)
                .addD(0)
                .build();
        } else {
            return new RevPidPropertyBuilder("Pivot Arm", Constants.DEFAULT_LOCK_PROPERTIES, pidController, 0)
                .addP(0.0045) // 0.0058
                .addD(0.045)
                .build();
        }
    }

    @Override
    public void periodic() {
        m_sparkPidProperties.updateIfChanged();
        m_wpiPidProperties.updateIfChanged();
        m_wpiFeedForward.updateIfChanged();

        m_networkTableEntries.updateLogs();

        m_pivotErrorAlert.checkAlerts();
    }

    public double jfdklsfjaklsdj() {
        return 0;
    }

    @Override
    public void simulationPeriodic() {
        m_pivotSimulator.update();
    }

    public void pivotArmUp() {
        m_pivotMotor.set(ARM_MOTOR_SPEED);
    }

    public void pivotArmDown() {
        m_pivotMotor.set(-ARM_MOTOR_SPEED);
    }

    public double getArmMotorSpeed() {
        return m_pivotMotor.getAppliedOutput();
    }

    public void pivotArmStop() {
        m_armAngleGoal = Double.MIN_VALUE;
        m_pivotMotor.set(0);
    }

    public final double getArmAngleDeg2() {
        return m_pivotMotorEncoder.getPosition();
    }

    public double getArmVelocityDegPerSec2() {
        return m_pivotMotorEncoder.getVelocity();
    }

    public double getArmAngleGoal() {
        return m_armAngleGoal;
    }

    public boolean isUpperLimitSwitchedPressed() {
        return !m_upperLimitSwitch.get();
    }

    public boolean isLowerLimitSwitchedPressed() {
        return !m_lowerLimitSwitch.get();
    }

    private boolean isMotionProfileFinished() {
        return m_wpiPidController.getSetpoint().equals(m_wpiPidController.getGoal());
    }

    public void pivotArmToAngle2(double pivotAngleGoal) {
        m_armAngleGoal = pivotAngleGoal;

        m_wpiPidController.calculate(getFeedbackAngleDeg(), pivotAngleGoal);

        TrapezoidProfile.State profileSetpointDegrees = m_wpiPidController.getSetpoint();
        m_profileVelocityGoalEntry.setNumber(profileSetpointDegrees.velocity);
        m_profilePositionGoalEntry.setNumber(profileSetpointDegrees.position);

        double feedForwardVolts = m_wpiFeedForward.calculate(
                Units.degreesToRadians(profileSetpointDegrees.position),
                Units.degreesToRadians(profileSetpointDegrees.velocity));
        m_pidArbitraryFeedForwardEntry.setNumber(feedForwardVolts);

        if (isMotionProfileFinished()) {
            m_sparkPidController.setReference(pivotAngleGoal, ControlType.kPosition, 0, feedForwardVolts);
        } else {
            m_pivotMotor.setVoltage(feedForwardVolts);
        }
    }

    public boolean isArmAtAngle() {
        return isArmAtAngle(m_armAngleGoal);
    }

    public boolean isArmAtAngle(double pivotAngleGoal) {
        return isArmAtAngle(pivotAngleGoal, ALLOWABLE_ERROR.getValue(), ALLOWABLE_VELOCITY_ERROR.getValue());
    }

    public boolean isArmAtAngle(double pivotAngleGoal, double allowableError, double allowableVelocityError) {
        double error = getFeedbackAngleDeg() - pivotAngleGoal;
        double velocity = getFeedbackVelocityDegPerSec();

        return isMotionProfileFinished() && Math.abs(error) <= allowableError && Math.abs(velocity) < allowableVelocityError;
    }

    public void clearStickyFaults() {
        m_pivotMotor.clearFaults();
    }


    public final void resetPivotEncoder(double angle) {
        SparkMaxUtil.autoRetry(() -> m_pivotMotorEncoder.setPosition(angle));
    }

    public static double getArmAngleForScoring(AutoPivotHeight pivotHeightType, GamePieceType gamePieceType) {
        double angle;

        switch (pivotHeightType) {
        case HIGH:
            if (gamePieceType == GamePieceType.CONE) {
                angle = ARM_CONE_HIGH_DEG;
            } else {
                angle = ARM_CUBE_HIGH_DEG;
            }
            break;
        case MEDIUM:
            if (gamePieceType == GamePieceType.CONE) {
                angle = ARM_CONE_MIDDLE_DEG;
            } else {
                angle = ARM_CUBE_MIDDLE_DEG;
            }
            break;
        case LOW:
        default:
            angle = ARM_SCORE_LOW_DEG;
            break;
        }
        return angle;
    }

    private void resetWpiPidController() {
        m_wpiPidController.reset(getFeedbackAngleDeg(), getFeedbackVelocityDegPerSec());
    }

    public boolean areEncodersSynced() {
        return Math.abs(getArmAngleDeg2() - getAbsoluteEncoderAngle2()) < 1;
    }


    private void setIdleMode(IdleMode idleMode) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        m_pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    ////////////////
    // Checklists
    ////////////////
    public Command createIsPivotMotorMovingChecklist() {
        return new SparkMaxMotorsMoveChecklist(this, m_pivotMotor, "Arm: Pivot motor", 1.0);
    }

    ///////////////////////
    // Command Factories
    ///////////////////////
    public Command createPivotToCoastModeCommand() {
        return this.runEnd(
            () -> setIdleMode(IdleMode.kCoast),
            () -> setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true).withName("Pivot to Coast");
    }

    public Command createResetPivotEncoderCommand() {
        return createResetPivotEncoderCommand(MIN_ANGLE_DEG);
    }

    public Command createResetPivotEncoderCommand(double angle) {
        return this.run(() -> resetPivotEncoder(angle))
            .ignoringDisable(true)
            .withName("Reset Pivot Encoder (" + angle + ")");
    }

    public Command createSyncEncoderToAbsoluteEncoderCommand() {
        return this.run(this::syncMotorEncoderToAbsoluteEncoder)
            .ignoringDisable(true)
            .withName("Reset Pivot Encoder (Abs Encoder Val)");
    }

    public Command createPivotUpCommand() {
        return this.runEnd(this::pivotArmUp, this::pivotArmStop).withName("Arm: Pivot Up");
    }

    public Command createPivotDownCommand() {
        return this.runEnd(this::pivotArmDown, this::pivotArmStop).withName("Arm: Pivot Down");
    }

    private Command createResetWpiPidControllerCommand() {
        return runOnce(this::resetWpiPidController);
    }

    private Command createBasePivotToAngleCommand(double angle, Runnable endBehavior, DoubleSupplier allowablePositionError, DoubleSupplier allowableVelocityError) {
        return createResetWpiPidControllerCommand()
            .andThen(runEnd(() -> pivotArmToAngle2(angle), endBehavior))
            .until(() -> isArmAtAngle(angle, allowablePositionError.getAsDouble(), allowableVelocityError.getAsDouble()))
            .withName("Pivot To " + angle);
    }

    private Command createBasePivotToAngleCommand(double angle, Runnable endBehavior, double allowablePositionError, double allowableVelocityError) {
        return createBasePivotToAngleCommand(angle, endBehavior, () -> allowablePositionError, () -> allowableVelocityError);
    }

    private Command createBasePivotToAngleCommand(double angle, Runnable endBehavior) {
        return createBasePivotToAngleCommand(angle, endBehavior, ALLOWABLE_ERROR::getValue, ALLOWABLE_VELOCITY_ERROR::getValue);
    }

    public Command createPivotToAngleAndHoldCommand(double angle) {
        return createBasePivotToAngleCommand(angle, () -> {})
            .withName("Arm to Angle And Hold" + angle);
    }

    public Command createPivotToAngleAndHoldCommand(double angle, double allowableError, double velocityAllowableError) {
        return createBasePivotToAngleCommand(angle, () -> {}, allowableError, velocityAllowableError)
            .withName("Arm to Angle And Hold" + angle);
    }

    public Command createPivotArmToAngleNonHoldCommand(double angle) {
        return createBasePivotToAngleCommand(angle, this::pivotArmStop)
            .withName("Arm to Angle" + angle);
    }

    public Command createMoveArmToPieceScorePositionAndHoldCommand(AutoPivotHeight height, GamePieceType gamePieceType) {
        double angle = getArmAngleForScoring(height, gamePieceType);
        return createPivotToAngleAndHoldCommand(angle).withName("Score [" + height + "," + gamePieceType + "] and Hold");
    }

    public Command createMoveArmToPieceScorePositionDifferenceAndHoldCommand(AutoPivotHeight height, GamePieceType gamePieceType, double heightChange) {
        double angle = getArmAngleForScoring(height, gamePieceType);
        return createPivotToAngleAndHoldCommand(angle + heightChange).withName("Score [" + height + "," + gamePieceType + "] and Hold");
    }

    public Command createGoHomeCommand() {
        return createPivotArmToAngleNonHoldCommand(HOME_ANGLE);
    }

    public Command createGoToGroundPickupCommand() {
        return createPivotArmToAngleNonHoldCommand(GROUND_PICKUP_ANGLE);
    }

    public Command createGoToGroundPickupAndHoldCommand() {
        return createPivotToAngleAndHoldCommand(GROUND_PICKUP_ANGLE);
    }

    public Command createGoToGroundPickupAndHoldCommand(double allowableError, double velocityAllowableError) {
        return createPivotToAngleAndHoldCommand(GROUND_PICKUP_ANGLE, allowableError, velocityAllowableError);
    }

    public Command createGoToHpPickupHoldCommand() {
        return createPivotToAngleAndHoldCommand(HUMAN_PLAYER_ANGLE);
    }

    public Command createGoToAutoNodePositionCommand(Supplier<AutoAimNodePositions> nodeSupplier) {
        return runEnd(() -> {
            AutoAimNodePositions position = nodeSupplier.get();
            if (position == null || position == AutoAimNodePositions.NONE) {
                return;
            }
            pivotArmToAngle2(position.getTargetPitch());
        }, this::pivotArmStop);
    }
}
