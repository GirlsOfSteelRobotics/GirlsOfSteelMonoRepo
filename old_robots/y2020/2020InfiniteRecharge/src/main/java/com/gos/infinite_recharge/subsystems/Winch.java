package com.gos.infinite_recharge.subsystems;

import com.gos.infinite_recharge.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {

    private final SparkMax m_motorA;
    private final SparkMax m_motorB;
    private final RelativeEncoder m_encoder;

    private final NetworkTable m_customNetworkTable;

    public Winch(boolean isBrushed) {
        m_motorA = new SparkMax(Constants.WINCH_A_SPARK, isBrushed ? MotorType.kBrushed : MotorType.kBrushless);
        SparkMaxConfig motorAConfig = new SparkMaxConfig();
        motorAConfig.idleMode(IdleMode.kBrake);
        m_motorA.setInverted(false);
        if (isBrushed) {
            m_encoder = m_motorA.getEncoder();
        } else {
            m_encoder = m_motorA.getEncoder();
        }

        m_motorB = new SparkMax(Constants.WINCH_B_SPARK, isBrushed ? MotorType.kBrushed : MotorType.kBrushless);
        SparkMaxConfig motorBConfig = new SparkMaxConfig();
        motorBConfig.idleMode(IdleMode.kBrake);
        m_motorB.setInverted(false);

        m_motorA.configure(motorAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorB.configure(motorBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_customNetworkTable = NetworkTableInstance.getDefault().getTable("SuperStructure/Winch");
    }

    public void wind() {
        m_motorA.set(0.8);
        m_motorB.set(0.8);
    }

    public void unwind() {
        m_motorA.set(-0.8);
        m_motorB.set(-0.8);
    }

    @Override
    public void periodic() {
        m_customNetworkTable.getEntry("Speed").setDouble(m_motorA.get());
        m_customNetworkTable.getEntry("Height").setDouble(m_encoder.getPosition());
    }

    public void stop() {
        m_motorA.set(0);
        m_motorB.set(0);
    }

}
