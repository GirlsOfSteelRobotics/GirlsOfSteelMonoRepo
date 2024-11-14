package com.gos.codelabs.pid.subsystems;

import com.gos.codelabs.pid.Constants;
import com.gos.codelabs.pid.Constants.ElevatorSimConstants;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ElevatorSubsystem extends SubsystemBase {
    public static final GosDoubleProperty GRAVITY_COMPENSATION = new GosDoubleProperty(false, "Elevator.GravityCompensationSpeed", 0);

    public static final double ALLOWABLE_POSITION_ERROR = .25;

    public enum Positions {
        LOW(Units.inchesToMeters(10)),
        MID(Units.inchesToMeters(35)),
        HIGH(Units.inchesToMeters(45));

        public final double m_heightMeters;

        Positions(double heightMeters) {
            m_heightMeters = heightMeters;
        }
    }


    private final SparkMax m_liftMotor;
    private final RelativeEncoder m_liftEncoder;
    private final DigitalInput m_lowerLimitSwitch;
    private final DigitalInput m_upperLimitSwitch;
    private final SparkClosedLoopController m_pidController;
    private final PidProperty m_pidProperty;
    private double m_desiredHeight;

    private ISimWrapper m_elevatorSim;

    public ElevatorSubsystem() {
        m_liftMotor = new SparkMax(Constants.CAN_LIFT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig liftConfig = new SparkMaxConfig();
        m_liftEncoder = m_liftMotor.getEncoder();
        m_pidController = m_liftMotor.getClosedLoopController();

        m_lowerLimitSwitch = new DigitalInput(Constants.DIO_LIFT_LOWER_LIMIT);
        m_upperLimitSwitch = new DigitalInput(Constants.DIO_LIFT_UPPER_LIMIT);

        m_pidProperty = new RevPidPropertyBuilder("Elevator", false, m_liftMotor, liftConfig, ClosedLoopSlot.kSlot0)
                .addP(0)
                .addFF(0)
                .addMaxAcceleration(0.1)
                .addMaxVelocity(0.1)
                .build();

        m_liftMotor.configure(liftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_liftMotor, ElevatorSimConstants.ELEVATOR_GEARBOX),
                    RevEncoderSimWrapper.create(m_liftMotor));
        }
    }

    @Override
    public void periodic() {
        m_pidProperty.updateIfChanged();
    }

    public boolean goToPosition(Positions position) {
        return goToPosition(position.m_heightMeters);
    }

    public boolean goToPosition(double position) {
        m_desiredHeight = position;
        m_pidController.setReference(position, ControlType.kMAXMotionPositionControl, 0, GRAVITY_COMPENSATION.getValue(), ArbFFUnits.kPercentOut);
        return false;
    }

    public boolean isAtLowerLimit() {
        return m_lowerLimitSwitch.get();
    }

    public boolean isAtUpperLimit() {
        return m_upperLimitSwitch.get();
    }

    public void setSpeed(double speed) {
        m_desiredHeight = -999;
        m_liftMotor.set(speed);
    }

    public double getHeightMeters() {
        return m_liftEncoder.getPosition();
    }

    public double getHeightInches() {
        return Units.metersToInches(getHeightMeters());
    }

    public double getVelocityMps() {
        return m_liftEncoder.getVelocity();
    }

    public double getVelocityInchesPerSec() {
        return Units.metersToInches(getVelocityMps());
    }

    public double getDesiredHeightMeters() {
        return m_desiredHeight;
    }

    public double getDesiredHeightInches() {
        return Units.metersToInches(getDesiredHeightMeters());
    }

    public double getMotorSpeed() {
        return m_liftMotor.getAppliedOutput();
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.update();
    }
}
