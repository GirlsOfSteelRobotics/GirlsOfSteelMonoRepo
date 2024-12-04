package com.gos.codelabs.basic_simulator.subsystems;

import com.gos.codelabs.basic_simulator.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.BaseDigitalInputWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;

public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {
    public static final double ALLOWABLE_POSITION_ERROR = Units.inchesToMeters(1);

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

    private ElevatorSimWrapper m_elevatorSim;

    private static final class ElevatorSimConstants {
        public static final double K_ELEVATOR_GEARING = 10.0;
        public static final double K_CARRIAGE_MASS = 4.0; // kg
        public static final double K_MIN_ELEVATOR_HEIGHT = -5.0;
        public static final double K_MAX_ELEVATOR_HEIGHT = Units.inchesToMeters(50e50);
        public static final DCMotor K_ELEVATOR_GEARBOX = DCMotor.getVex775Pro(4);
        public static final double K_ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(2.0);
    }

    public ElevatorSubsystem() {
        m_liftMotor = new SparkMax(Constants.CAN_LIFT_MOTOR, MotorType.kBrushless);
        m_liftEncoder = m_liftMotor.getEncoder();

        m_lowerLimitSwitch = new DigitalInput(Constants.DIO_LIFT_LOWER_LIMIT);
        m_upperLimitSwitch = new DigitalInput(Constants.DIO_LIFT_UPPER_LIMIT);

        if (RobotBase.isSimulation()) {
            ElevatorSim sim = new ElevatorSim(
                    ElevatorSimConstants.K_ELEVATOR_GEARBOX,
                    ElevatorSimConstants.K_ELEVATOR_GEARING,
                    ElevatorSimConstants.K_CARRIAGE_MASS,
                    ElevatorSimConstants.K_ELEVATOR_DRUM_RADIUS,
                    ElevatorSimConstants.K_MIN_ELEVATOR_HEIGHT,
                    ElevatorSimConstants.K_MAX_ELEVATOR_HEIGHT, true, 0);

            m_elevatorSim = new ElevatorSimWrapper(sim,
                    new RevMotorControllerSimWrapper(m_liftMotor),
                    RevEncoderSimWrapper.create(m_liftMotor));
            m_elevatorSim.setLowerLimitSwitch(new BaseDigitalInputWrapper(new DIOSim(m_lowerLimitSwitch)::setValue));
            m_elevatorSim.setUpperLimitSwitch(new BaseDigitalInputWrapper(new DIOSim(m_upperLimitSwitch)::setValue));
        }
    }

    @Override
    public void close() {
        m_liftMotor.close();
        m_lowerLimitSwitch.close();
        m_upperLimitSwitch.close();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.update();
    }

    public boolean goToPosition(double position) {
        // TODO implement
        return false;
    }

    public boolean isAtLowerLimit() {
        // TODO implement
        return false;
    }

    public boolean isAtUpperLimit() {
        // TODO implement
        return false;
    }

    public void stop() {
        // TODO implement
    }

    public void setSpeed(double speed) {
        // TODO implement
    }

    public double getHeight() {
        // TODO implement
        return 0;
    }
}
