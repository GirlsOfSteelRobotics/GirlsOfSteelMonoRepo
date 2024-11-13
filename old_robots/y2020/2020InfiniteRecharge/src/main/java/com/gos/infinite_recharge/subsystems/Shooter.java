package com.gos.infinite_recharge.subsystems;

import com.gos.infinite_recharge.Constants;
import com.gos.lib.properties.GosDoubleProperty;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private static final double SHOOTER_KP = 0.0005;
    private static final double SHOOTER_KFF = 0.000087; //1 / 10600.0;
    private static final double SHOOTER_KD = 0.0001;

    private static final double ALLOWABLE_ERROR_PERCENT = 1;


    private final SparkMax m_master;
    private final SparkMax m_follower;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;

    private final Limelight m_limelight;

    private double m_goalRPM;

    private final NetworkTable m_customNetworkTable;

    private final GosDoubleProperty m_dashboardKp;
    private final GosDoubleProperty m_dashboardKff;

    private final GenericEntry m_isAtShooterSpeedEntry;

    private ISimWrapper m_simulator;

    public Shooter(ShuffleboardTab driveDisplayTab, Limelight limelight) {
        m_master = new SparkMax(Constants.SHOOTER_SPARK_A, MotorType.kBrushed);
        m_follower = new SparkMax(Constants.SHOOTER_SPARK_B, MotorType.kBrushed);
        m_encoder  = m_master.getEncoder();
        m_pidController = m_master.getClosedLoopController();

        SparkMaxConfig masterConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        m_dashboardKp = new GosDoubleProperty(false, "shooter_kp", SHOOTER_KP);
        m_dashboardKff = new GosDoubleProperty(false, "shooter_kff", SHOOTER_KFF);

        m_limelight = limelight;


        masterConfig.encoder.inverted(true);

        masterConfig.smartCurrentLimit(Constants.SPARK_MAX_CURRENT_LIMIT);
        m_master.setInverted(false);
        followerConfig.follow(m_master, true);

        masterConfig.closedLoop.p(SHOOTER_KP);
        masterConfig.closedLoop.velocityFF(SHOOTER_KFF);
        masterConfig.closedLoop.d(SHOOTER_KD);

        m_master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_customNetworkTable = NetworkTableInstance.getDefault().getTable("SuperStructure/Shooter");
        NetworkTableInstance.getDefault().getTable("SuperStructure").getEntry(".type").setString("SuperStructure");

        m_isAtShooterSpeedEntry = driveDisplayTab.add("Shooter At Speed", isAtFullSpeed()) // NOPMD
            .withSize(4, 1)
            .withPosition(0, 0)
            .getEntry();

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getVex775Pro(2);
            LinearSystem<N1, N1, N1> plant =
                LinearSystemId.createFlywheelSystem(gearbox, .008, 1.66);
            FlywheelSim flywheelSim = new FlywheelSim(plant, gearbox);

            m_simulator = new FlywheelSimWrapper(flywheelSim,
                    new RevMotorControllerSimWrapper(m_master),
                    RevEncoderSimWrapper.create(m_master));
        }
    }


    public void setRPM(final double rpm) {
        m_goalRPM = rpm;
        m_pidController.setReference(rpm, ControlType.kVelocity);
        //double targetVelocityUnitsPer100ms = rpm * 4096 / 600;
        //m_master.set(1.00 /*targetVelocityUnitsPer100ms*/);
        m_limelight.turnLimelightOn();
    }

    // public void Limelight() {
    //     m_limelight - limelight

    @Override
    public void periodic() {
        double rpm = m_encoder.getVelocity();
        SmartDashboard.putNumber("RPM", rpm);
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        m_customNetworkTable.getEntry("Speed").setDouble(m_master.get());
        m_customNetworkTable.getEntry("Current RPM").setDouble(rpm);
        m_customNetworkTable.getEntry("Goal RPM").setDouble(m_goalRPM);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(m_dashboardKp.getValue());
        config.closedLoop.velocityFF(m_dashboardKff.getValue());
        m_master.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // System.out.println("kp: " + m_dashboardKp.getValue() + ", " + m_dashboardKff.getValue() + " goal: " + m_goalRPM + "== " + rpm);

        m_isAtShooterSpeedEntry.setBoolean(isAtFullSpeed());
    }

    public boolean isAtFullSpeed() {
        double currentRPM = m_encoder.getVelocity();
        double percentError = (m_goalRPM - currentRPM) / m_goalRPM * 100;
        return Math.abs(percentError) <= ALLOWABLE_ERROR_PERCENT;
    }

    public void stop() {
        m_master.set(0);
        m_limelight.turnLimelightOff();
        //m_pidController.setReference(0, ControlType.kVelocity);
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
    }
}
