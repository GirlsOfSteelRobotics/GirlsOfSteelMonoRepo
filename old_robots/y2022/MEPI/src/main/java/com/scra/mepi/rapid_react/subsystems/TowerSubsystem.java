// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.scra.mepi.rapid_react.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.scra.mepi.rapid_react.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TowerSubsystem extends SubsystemBase {
    private final SparkMax m_towerMotor;
    private final SparkMax m_towerKicker;
    private final DigitalInput m_beamBreak = new DigitalInput(9);

    /**
     * Creates a new Tower.
     */
    public TowerSubsystem() {
        m_towerMotor = new SparkMax(Constants.TOWER_SPARK, MotorType.kBrushless);
        m_towerKicker = new SparkMax(Constants.TOWER_KICKER_SPARK, MotorType.kBrushless);
        SparkMaxConfig towerMotorConfig = new SparkMaxConfig();
        SparkMaxConfig towerKickerConfig = new SparkMaxConfig();
        towerMotorConfig.smartCurrentLimit(30);
        towerKickerConfig.smartCurrentLimit(30);
        towerMotorConfig.idleMode(IdleMode.kBrake);
        towerKickerConfig.idleMode(IdleMode.kCoast);

        m_towerKicker.configure(towerKickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_towerMotor.configure(towerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("beam break", m_beamBreak.get());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setTowerSpeed(double speed) {
        m_towerMotor.set(speed);
    }

    public boolean getBeamBreak() {
        return m_beamBreak.get();
    }

    public void setKickerSpeed(double speed) {
        m_towerKicker.set(speed);
    }
}
