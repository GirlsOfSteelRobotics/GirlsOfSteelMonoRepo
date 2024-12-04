package com.gos.rapidreact.subsystems;

import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.gos.rapidreact.Constants;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SingleJointedArmSimWrapper;

@SuppressWarnings({"PMD.TooManyMethods", "PMD.TooManyFields"})
public class CollectorSubsystem extends SubsystemBase {
    private static final double ROLLER_SPEED = 0.5;
    private static final double PIVOT_SPEED = .2;
    public static final double ALLOWABLE_ERROR_DEG = 1;
    public static final GosDoubleProperty GRAVITY_OFFSET = new GosDoubleProperty(false, "Gravity Offset", 0);
    private static final double GEARING =  252.0;
    private static final double J_KG_METERS_SQUARED = 1;
    private static final double ARM_LENGTH_METERS = Units.inchesToMeters(16);
    private static final double MIN_ANGLE_RADS = 0;
    private static final double MAX_ANGLE_RADS = Math.PI / 2;
    private static final boolean SIMULATE_GRAVITY = true;

    // TODO play with these numbers for optimal ball pickup
    public static final double UP_ANGLE = 80;
    public static final double DOWN_ANGLE_TELE = RobotBase.isReal() ? -1 : 0;
    public static final double DOWN_ANGLE_AUTO = RobotBase.isReal() ? 7 : 0;

    // From SysId
    private static final double PIVOT_KS = 0.18148;
    //private static final double PIVOT_KV = 0.12495;
    //private static final double PIVOT_KA = 0.0051913;

    private final SparkMax m_roller;
    private final SparkMax m_pivotLeft;
    private final SparkMax m_pivotRight;

    private final DigitalInput m_indexSensor;

    private final RelativeEncoder m_pivotNeoEncoderLeft;
    private final RelativeEncoder m_pivotNeoEncoderRight;

    private final AbsoluteEncoder m_pivotExternalEncoderLeft;
    private final AbsoluteEncoder m_pivotExternalEncoderRight;

    private final PidProperty m_pivotPIDLeft;
    private final SparkClosedLoopController m_pidControllerLeft;

    private final PidProperty m_pivotPIDRight;
    private final SparkClosedLoopController m_pidControllerRight;

    private SingleJointedArmSimWrapper m_leftSimulator;
    private SingleJointedArmSimWrapper m_rightSimulator;

    private final DigitalInput m_lowerLimitSwitch;
    private final DigitalInput m_upperLimitSwitch;

    // Logging
    private final NetworkTableEntry m_leftIntakeAngleNeoEncoderEntry;
    private final NetworkTableEntry m_leftIntakeVelocityEntry;
    private final NetworkTableEntry m_rightIntakeAngleNeoEncoderEntry;
    private final NetworkTableEntry m_rightIntakeVelocityEntry;
    private final NetworkTableEntry m_lowerIntakeSwitchPressedEntry;
    private final NetworkTableEntry m_upperIntakeSwitchPressedEntry;
    private final NetworkTableEntry m_intakeAngleGoalEntry;
    private final NetworkTableEntry m_leftIntakeAngleExternalEncoderEntry;
    private final NetworkTableEntry m_rightIntakeAngleExternalEncoderEntry;

    private final NetworkTableEntry m_leftGravityOffsetVoltage;
    private final NetworkTableEntry m_rightGravityOffsetVoltage;

    public CollectorSubsystem() {
        m_roller = new SparkMax(Constants.COLLECTOR_ROLLER, MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);

        m_pivotLeft = new SparkMax(Constants.COLLECTOR_PIVOT_LEFT, MotorType.kBrushless);
        SparkMaxConfig pivotLeftConfig = new SparkMaxConfig();
        pivotLeftConfig.idleMode(IdleMode.kBrake);
        m_pivotLeft.setInverted(true);

        m_pivotRight = new SparkMax(Constants.COLLECTOR_PIVOT_RIGHT, MotorType.kBrushless);
        SparkMaxConfig pivotRightConfig = new SparkMaxConfig();
        pivotRightConfig.idleMode(IdleMode.kBrake);

        m_pivotNeoEncoderLeft = m_pivotLeft.getEncoder();
        pivotLeftConfig.encoder.positionConversionFactor(360.0 / GEARING);
        pivotLeftConfig.encoder.velocityConversionFactor(360.0 / GEARING / 60.0);

        m_pivotNeoEncoderRight = m_pivotRight.getEncoder();
        pivotRightConfig.encoder.positionConversionFactor(360.0 / GEARING);
        pivotRightConfig.encoder.velocityConversionFactor(360.0 / GEARING / 60.0);

        m_pivotExternalEncoderLeft = m_pivotLeft.getAbsoluteEncoder();
        pivotLeftConfig.alternateEncoder.positionConversionFactor(360.0);
        pivotLeftConfig.alternateEncoder.velocityConversionFactor(360.0 / 60.0);

        m_pivotExternalEncoderRight = m_pivotRight.getAbsoluteEncoder();
        pivotRightConfig.alternateEncoder.velocityConversionFactor(360.0);
        pivotRightConfig.alternateEncoder.velocityConversionFactor(360.0 / 60.0);

        m_indexSensor = new DigitalInput(Constants.INTAKE_INDEX_SENSOR);

        m_pidControllerLeft = m_pivotLeft.getClosedLoopController();
        m_pidControllerRight = m_pivotRight.getClosedLoopController();

        m_lowerLimitSwitch = new DigitalInput(Constants.INTAKE_LOWER_LIMIT_SWITCH);
        m_upperLimitSwitch = new DigitalInput(Constants.INTAKE_UPPER_LIMIT_SWITCH);

        m_pivotPIDLeft = setupPidValues(m_pidControllerLeft);
        m_pivotPIDRight = setupPidValues(m_pidControllerRight);

        resetPivotEncoder();

        m_roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pivotLeft.configure(pivotLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pivotRight.configure(pivotRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("CollectorSubsystem");
        m_leftIntakeAngleNeoEncoderEntry = loggingTable.getEntry("Left Intake Neo (deg)");
        m_leftIntakeVelocityEntry = loggingTable.getEntry("Left Intake (dps)");
        m_rightIntakeAngleNeoEncoderEntry = loggingTable.getEntry("Right Intake Neo (deg)");
        m_rightIntakeVelocityEntry = loggingTable.getEntry("Right Intake (dps)");
        m_lowerIntakeSwitchPressedEntry = loggingTable.getEntry("Lower Limit Switch");
        m_upperIntakeSwitchPressedEntry = loggingTable.getEntry("Upper Limit Switch");
        m_intakeAngleGoalEntry = loggingTable.getEntry("Angle Goal");
        m_leftGravityOffsetVoltage = loggingTable.getEntry("Left Gravity Offset (Voltage)");
        m_rightGravityOffsetVoltage = loggingTable.getEntry("Right Gravity Offset (Voltage)");
        m_leftIntakeAngleExternalEncoderEntry = loggingTable.getEntry("Left Intake External (raw)");
        m_rightIntakeAngleExternalEncoderEntry = loggingTable.getEntry("Right Intake External (raw)");

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNeo550(1);
            SingleJointedArmSim armSim = new SingleJointedArmSim(gearbox, GEARING, J_KG_METERS_SQUARED,
                ARM_LENGTH_METERS, MIN_ANGLE_RADS, MAX_ANGLE_RADS, SIMULATE_GRAVITY, 0);
            m_leftSimulator = new SingleJointedArmSimWrapper(armSim, new RevMotorControllerSimWrapper(m_pivotLeft),
                RevEncoderSimWrapper.create(m_pivotLeft), true);
            m_rightSimulator = new SingleJointedArmSimWrapper(armSim, new RevMotorControllerSimWrapper(m_pivotRight),
                RevEncoderSimWrapper.create(m_pivotRight), true);
        }
    }

    private PidProperty setupPidValues(SparkClosedLoopController pidController) {
        return new RevPidPropertyBuilder("Collector", false, pidController, 0)
            .addP(0) //0.20201
            .addI(0)
            .addD(0)
            .addFF(0)
            .addMaxVelocity(Units.inchesToMeters(0))
            .addMaxAcceleration(Units.inchesToMeters(0))
            .build();
    }

    @Override
    public void periodic() {
        m_leftIntakeAngleNeoEncoderEntry.setNumber(getIntakeLeftAngleDegreesNeoEncoder());
        m_leftIntakeVelocityEntry.setNumber(m_pivotNeoEncoderLeft.getVelocity());
        m_rightIntakeAngleNeoEncoderEntry.setNumber(getIntakeRightAngleDegreesNeoEncoder());
        m_rightIntakeVelocityEntry.setNumber(m_pivotNeoEncoderRight.getVelocity());
        m_lowerIntakeSwitchPressedEntry.setBoolean(lowerLimitSwitchPressed());
        m_upperIntakeSwitchPressedEntry.setBoolean(upperLimitSwitchPressed());
        m_leftIntakeAngleExternalEncoderEntry.setNumber(getIntakeLeftAngleRawExternalEncoder());
        m_rightIntakeAngleExternalEncoderEntry.setNumber(getIntakeRightAngleRawExternalEncoder());

        m_pivotPIDLeft.updateIfChanged();
        m_pivotPIDRight.updateIfChanged();

        SmartDashboard.putNumber("Left Intake External (raw)", getIntakeLeftAngleRawExternalEncoder());
        SmartDashboard.putNumber("Right Intake External (raw)", getIntakeRightAngleRawExternalEncoder());
    }

    public void collectorDown() {
        if (lowerLimitSwitchPressed()) {
            m_pivotLeft.set(0);
            m_pivotRight.set(0);
        }

        else {
            m_pivotLeft.set(-PIVOT_SPEED);
            m_pivotRight.set(-PIVOT_SPEED);
        }
    }

    public void collectorUp() {
        if (upperLimitSwitchPressed()) {
            m_pivotLeft.set(0);
            m_pivotRight.set(0);
        }

        else {
            m_pivotLeft.set(PIVOT_SPEED);
            m_pivotRight.set(PIVOT_SPEED);
        }
    }

    public boolean lowerLimitSwitchPressed() {
        return !m_lowerLimitSwitch.get();
    }

    public boolean upperLimitSwitchPressed() {
        return !m_upperLimitSwitch.get();
    }

    public void rollerIn() {
        m_roller.set(ROLLER_SPEED);
    }

    public void rollerOut() {
        m_roller.set(-ROLLER_SPEED);
    }

    public void rollerStop() {
        m_roller.set(0);
    }

    public void pivotStop() {
        m_pivotLeft.set(0);
        m_pivotRight.set(0);
    }

    /**
     * gets pivot point of collector to given angle using pid
     * @param pivotAngleDegreesGoal *IN DEGREES*
     */
    public boolean collectorToAngle(double pivotAngleDegreesGoal) {
        m_intakeAngleGoalEntry.setNumber(pivotAngleDegreesGoal);

        if (pivotAngleDegreesGoal < getIntakeLeftAngleDegreesNeoEncoder() && lowerLimitSwitchPressed()) {
            pivotStop();
            return true;
        }

        if (pivotAngleDegreesGoal > getIntakeLeftAngleDegreesNeoEncoder() && upperLimitSwitchPressed()) {
            pivotStop();
            return true;
        }

        else {
            double errorLeft = pivotAngleDegreesGoal - getIntakeLeftAngleDegreesNeoEncoder();
            double errorRight = pivotAngleDegreesGoal - getIntakeRightAngleDegreesNeoEncoder();

            double gravityOffsetLeft = Math.cos(getIntakeLeftAngleRadians()) * GRAVITY_OFFSET.getValue();
            double gravityOffsetRight = Math.cos(getIntakeRightAngleRadians()) * GRAVITY_OFFSET.getValue();
            double staticFrictionLeft = PIVOT_KS * Math.signum(errorLeft);
            double staticFrictionRight = PIVOT_KS * Math.signum(errorRight);
            double arbFeedforwardLeft = gravityOffsetLeft + staticFrictionLeft;
            double arbFeedforwardRight = gravityOffsetRight + staticFrictionRight;
            m_pidControllerLeft.setReference(pivotAngleDegreesGoal, ControlType.kSmartMotion, 0, arbFeedforwardLeft);
            m_pidControllerRight.setReference(pivotAngleDegreesGoal, ControlType.kSmartMotion, 0, arbFeedforwardRight);

            m_leftGravityOffsetVoltage.setNumber(gravityOffsetLeft);
            m_rightGravityOffsetVoltage.setNumber(gravityOffsetRight);
            double error = Math.abs(pivotAngleDegreesGoal - getIntakeLeftAngleDegreesNeoEncoder());
            return error < CollectorSubsystem.ALLOWABLE_ERROR_DEG;
        }

    }

    public double getIntakeLeftAngleRadians() {
        return Math.toRadians(getIntakeLeftAngleDegreesNeoEncoder());
    }

    public double getIntakeRightAngleRadians() {
        return Math.toRadians(getIntakeRightAngleDegreesNeoEncoder());
    }

    public double getIntakeLeftAngleDegreesNeoEncoder() { //for leader
        return m_pivotNeoEncoderLeft.getPosition();
    }

    public double getIntakeRightAngleDegreesNeoEncoder() {
        return m_pivotNeoEncoderRight.getPosition();
    }

    public double getIntakeLeftAngleRawExternalEncoder() {
        return m_pivotExternalEncoderLeft.getPosition();
    }

    public double getIntakeRightAngleRawExternalEncoder() {
        return m_pivotExternalEncoderRight.getPosition();
    }

    public double getPivotSpeed() {
        return m_pivotLeft.getAppliedOutput();
    }

    public double getRollerSpeed() {
        return m_roller.getAppliedOutput();
    }

    public void tuneGravityOffset() {
        m_pivotLeft.setVoltage(GRAVITY_OFFSET.getValue());
    }

    @Override
    public void simulationPeriodic() {
        m_leftSimulator.update();
        m_rightSimulator.update();
    }

    public final void resetPivotEncoder() {
        m_pivotNeoEncoderLeft.setPosition(90);
        m_pivotNeoEncoderRight.setPosition(90);
    }

    public boolean getIndexSensor() {
        return !m_indexSensor.get();
    }
}
