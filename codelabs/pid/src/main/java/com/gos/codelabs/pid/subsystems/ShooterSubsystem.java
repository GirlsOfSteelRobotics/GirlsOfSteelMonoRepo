/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.gos.codelabs.pid.subsystems;

import com.gos.codelabs.pid.Constants;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax m_wheelMotor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;
    private final PidProperty m_pidProperty;
    private double m_desiredRpm;
    private ISimWrapper m_simulator;

    public ShooterSubsystem() {
        m_wheelMotor = new SparkMax(Constants.CAN_SPINNING_MOTOR, MotorType.kBrushless);
        SparkMaxConfig wheelConfig = new SparkMaxConfig();
        m_encoder = m_wheelMotor.getEncoder();
        m_pidController = m_wheelMotor.getClosedLoopController();

        m_pidProperty = new RevPidPropertyBuilder("Shooter", false, m_wheelMotor, wheelConfig, ClosedLoopSlot.kSlot0)
                .addP(0)
                .addFF(0)
                .build();

        m_wheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (RobotBase.isSimulation()) {
            m_simulator = new FlywheelSimWrapper(Constants.FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_wheelMotor),
                    RevEncoderSimWrapper.create(m_wheelMotor));
        }
    }

    public void setPercentOutput(double percentOutput) {
        m_wheelMotor.set(percentOutput);
    }

    public void spinAtRpm(double rpm) {
        m_desiredRpm = rpm;
        m_pidController.setReference(rpm, ControlType.kVelocity);
    }

    public double getRpm() {
        return m_encoder.getVelocity();
    }

    public double getDesiredRpm() {
        return m_desiredRpm;
    }

    public boolean isAtRpm(double rpm) {
        return Math.abs(getRpm() - rpm) < 10;
    }

    public double getMotorSpeed() {
        return m_wheelMotor.getAppliedOutput();
    }

    public void stop() {
        m_desiredRpm = -999;
        m_wheelMotor.set(0);
    }

    @Override
    public void periodic() {
        m_pidProperty.updateIfChanged();
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
    }
}
