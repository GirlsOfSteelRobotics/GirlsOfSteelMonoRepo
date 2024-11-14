package com.gos.rapidreact.subsystems;


import com.gos.lib.DeadbandHelper;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.gos.rapidreact.Constants;
import com.gos.rapidreact.subsystems.utils.ShooterLookupTable;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

@SuppressWarnings("PMD.TooManyFields")
public class ShooterSubsystem extends SubsystemBase {

    //variables for the two NEO Brushless Motors
    public static final double SHOOTER_ALLOWABLE_ERROR = 50.0;
    public static final double FENDER_RPM_LOW = 700;
    public static final double TARMAC_EDGE_RPM_LOW = 800;
    public static final double TARMAC_EDGE_RPM_HIGH = 1750;
    public static final double DEFAULT_SHOOTER_RPM = FENDER_RPM_LOW;
    public static final double MAX_SHOOTER_RPM = 3200;

    private static final double ROLLER_ALLOWABLE_ERROR = 100.0;
    private static final double ROLLER_SHOOTER_RPM_PROPORTION = 1.3;

    private final SparkMax m_leader;
    private final RelativeEncoder m_shooterEncoder;
    private final PidProperty m_shooterPid;
    private final SparkClosedLoopController m_shooterPidController;
    private final ShooterLookupTable m_shooterTable;
    private double m_shooterGoalRpm = Double.MAX_VALUE;
    private double m_backspinGoalRpm = Double.MAX_VALUE;

    private final SparkMax m_roller;
    private final RelativeEncoder m_rollerEncoder;
    private final PidProperty m_rollerPid;
    private final SparkClosedLoopController m_rollerPidController;

    private ISimWrapper m_shooterSimulator;
    private ISimWrapper m_backspinSimulator;

    private final DeadbandHelper m_counter;

    // Logging
    private final NetworkTableEntry m_shooterRpmEntry;
    private final NetworkTableEntry m_backspinRpmEntry;
    private final NetworkTableEntry m_shooterGoalRpmEntry;
    private final NetworkTableEntry m_backspinGoalRpmEntry;
    private final NetworkTableEntry m_shooterAtSpeedEntry;

    public ShooterSubsystem() {
        m_leader = new SparkMax(Constants.SHOOTER_LEADER_SPARK, MotorType.kBrushless);
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);

        m_roller = new SparkMax(Constants.SHOOTER_ROLLER, MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);
        m_roller.setInverted(true);

        //true because the motors are facing each other and in order to do the same thing, they would have to spin in opposite directions
        m_shooterEncoder = m_leader.getEncoder();
        m_rollerEncoder = m_roller.getEncoder();

        m_shooterPidController = m_leader.getClosedLoopController();
        m_shooterPid = new RevPidPropertyBuilder("Shooter PID", false, m_leader, leaderConfig, ClosedLoopSlot.kSlot0)
            .addP(0.003)
            .addD(0.000055)
            .addFF(0.000176)
            .build();

        m_rollerPidController = m_roller.getClosedLoopController();
        m_rollerPid = new RevPidPropertyBuilder("Roller PID", false, m_roller, rollerConfig, ClosedLoopSlot.kSlot0)
            .addP(0)
            .addD(0)
            .addFF(0)
            .build();

        m_shooterTable = new ShooterLookupTable();

        m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Shooter");
        m_shooterRpmEntry = loggingTable.getEntry("Shooter RPM");
        m_backspinRpmEntry = loggingTable.getEntry("Backspin RPM");
        m_shooterGoalRpmEntry = loggingTable.getEntry("Shooter Goal");
        m_backspinGoalRpmEntry = loggingTable.getEntry("Backspin Goal");
        m_shooterAtSpeedEntry = loggingTable.getEntry("At Speed");

        if (RobotBase.isSimulation()) {
            DCMotor shooterGearbox = DCMotor.getNeo550(2);
            LinearSystem<N1, N1, N1> shooterPlant =
                LinearSystemId.createFlywheelSystem(shooterGearbox, 0.01, 1.0);
            FlywheelSim shooterFlywheelSim = new FlywheelSim(shooterPlant, shooterGearbox);
            m_shooterSimulator = new FlywheelSimWrapper(shooterFlywheelSim,
                new RevMotorControllerSimWrapper(m_leader, shooterGearbox),
                RevEncoderSimWrapper.create(m_leader));

            DCMotor backspinGearbox = DCMotor.getNeo550(2);
            LinearSystem<N1, N1, N1> backspinPlant =
                LinearSystemId.createFlywheelSystem(shooterGearbox, 0.01, 1.0);
            FlywheelSim backspinFlywheelSim = new FlywheelSim(backspinPlant, backspinGearbox);
            m_backspinSimulator = new FlywheelSimWrapper(backspinFlywheelSim,
                new RevMotorControllerSimWrapper(m_roller, shooterGearbox),
                RevEncoderSimWrapper.create(m_roller));
        }

        m_counter = new DeadbandHelper(10);
    }

    @Override
    public void periodic() {
        m_shooterPid.updateIfChanged();
        m_rollerPid.updateIfChanged();
        double shooterError = Math.abs(m_shooterGoalRpm - getEncoderVelocity());
        double rollerError = Math.abs(ROLLER_SHOOTER_RPM_PROPORTION * m_shooterGoalRpm - getRollerVelocity());
        m_counter.setIsGood(shooterError < SHOOTER_ALLOWABLE_ERROR && rollerError < ROLLER_ALLOWABLE_ERROR);


        m_shooterRpmEntry.setNumber(getEncoderVelocity());
        m_backspinRpmEntry.setNumber(getRollerVelocity());
        m_shooterGoalRpmEntry.setNumber(m_shooterGoalRpm);
        m_backspinGoalRpmEntry.setNumber(m_backspinGoalRpm);
        m_shooterAtSpeedEntry.setBoolean(m_counter.isFinished());
    }

    public void setShooterRpmPIDSpeed(double rpm) {
        m_shooterGoalRpm = rpm;
        m_backspinGoalRpm = rpm * ROLLER_SHOOTER_RPM_PROPORTION;

        m_shooterPidController.setReference(rpm, ControlType.kVelocity);
        m_rollerPidController.setReference(ROLLER_SHOOTER_RPM_PROPORTION * rpm, ControlType.kVelocity);

    }

    public boolean isShooterAtSpeed() {
        return m_counter.isFinished();
    }

    public double getRollerVelocity() {
        return m_rollerEncoder.getVelocity();
    }

    public void setShooterSpeed(double speed) {
        m_leader.set(speed);
        m_roller.set(speed * ROLLER_SHOOTER_RPM_PROPORTION);

        m_shooterGoalRpm = Double.MAX_VALUE;
        m_backspinGoalRpm = Double.MAX_VALUE;
    }

    public double getEncoderVelocity() {
        return m_shooterEncoder.getVelocity();
    }

    public double getShooterSpeed() {
        return m_leader.getAppliedOutput();
    }

    public void rpmForDistance(double distance) {
        setShooterRpmPIDSpeed(m_shooterTable.getVelocityTable(distance));
    }

    @Override
    public void simulationPeriodic() {
        m_shooterSimulator.update();
        m_backspinSimulator.update();
    }

}
