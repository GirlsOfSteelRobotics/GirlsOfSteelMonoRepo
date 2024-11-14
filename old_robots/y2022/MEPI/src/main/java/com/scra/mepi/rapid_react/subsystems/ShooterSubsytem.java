// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.scra.mepi.rapid_react.subsystems;

import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
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
import com.scra.mepi.rapid_react.Constants;
import com.scra.mepi.rapid_react.ShooterLookupTable;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;
import org.snobotv2.sim_wrappers.InstantaneousMotorSim;

public class ShooterSubsytem extends SubsystemBase {
    private static final double AFTER_ENCODER_REDUCTION = 0.5;

    public static final double FENDER_RPM = 1500;

    /**
     * Creates a new Shooter.
     */
    private final SparkMax m_shooterMotor;

    private final SparkMax m_hoodMotor;

    private final ShooterLookupTable m_shooterLookupTable;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;
    private final PidProperty m_pidProperties;
    private final GosDoubleProperty m_tunableAllowableError = new GosDoubleProperty(false, "Shooter(AllowableError))", 50);

    // Simulation
    private ISimWrapper m_shooterSimulator;
    private ISimWrapper m_hoodSimulator;

    public ShooterSubsytem() {
        m_shooterMotor = new SparkMax(Constants.SHOOTER_SPARK, MotorType.kBrushless);
        SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

        m_hoodMotor = new SparkMax(Constants.SHOOTER_HOOD_SPARK, MotorType.kBrushless);
        SparkMaxConfig hoodMotorConfig = new SparkMaxConfig();

        shooterMotorConfig.smartCurrentLimit(50);
        hoodMotorConfig.smartCurrentLimit(30);
        m_shooterLookupTable = new ShooterLookupTable();
        m_encoder = m_shooterMotor.getEncoder();
        m_pidController = m_shooterMotor.getClosedLoopController();
        m_pidProperties = new RevPidPropertyBuilder("Shooter", false, m_shooterMotor, shooterMotorConfig, ClosedLoopSlot.kSlot0)
            .addP(0)
            .addI(0)
            .addD(0)
            .addFF(0.00045)
            .build();
        shooterMotorConfig.idleMode(IdleMode.kCoast);
        m_shooterMotor.setInverted(true);
        m_shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            DCMotor gearbox = DCMotor.getNeo550(2);
            LinearSystem<N1, N1, N1> plant =
                LinearSystemId.createFlywheelSystem(gearbox, 0.01, 1.0);
            FlywheelSim shooterFlywheelSim = new FlywheelSim(plant, gearbox);
            m_shooterSimulator = new FlywheelSimWrapper(shooterFlywheelSim,
                new RevMotorControllerSimWrapper(m_shooterMotor),
                RevEncoderSimWrapper.create(m_shooterMotor));

            m_hoodSimulator = new InstantaneousMotorSim(
                new RevMotorControllerSimWrapper(m_hoodMotor),
                RevEncoderSimWrapper.create(m_hoodMotor),
                1
            );
        }
    }

    @Override
    public void periodic() {
        m_pidProperties.updateIfChanged();
        SmartDashboard.putNumber("shooterRpm", getRPM());
    }

    @Override
    public void simulationPeriodic() {
        m_hoodSimulator.update();
        m_shooterSimulator.update();
    }

    public void setPidRpm(double rpm) {
        m_pidController.setReference(rpm, ControlType.kVelocity);
    }

    public boolean checkAtSpeed(double goal) {
        double error = Math.abs(goal - getRPM());
        return m_tunableAllowableError.getValue() > error;
    }

    public double getRPM() {
        return m_encoder.getVelocity() * AFTER_ENCODER_REDUCTION;
    }

    public void shootFromDistance(double distance) {
        setPidRpm(m_shooterLookupTable.getRpmTable(distance));
    }

    public void setShooterSpeed(double speed) {
        m_shooterMotor.set(speed);
    }
}
