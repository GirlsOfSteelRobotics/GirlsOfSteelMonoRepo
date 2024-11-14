package com.gos.lib.rev.properties.pid;

import com.gos.lib.properties.pid.IPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class RevPidPropertyBuilder extends PidProperty.Builder implements IPidPropertyBuilder {
    private final SparkBase m_motor;
    private final SparkBaseConfig m_parentConfig;
    private final ClosedLoopConfig m_closedLoopConfig;
    private final ClosedLoopConfig.ClosedLoopSlot m_slot;

    public RevPidPropertyBuilder(String baseName, boolean isConstant, SparkBase motor, SparkBaseConfig parentConfig, ClosedLoopConfig.ClosedLoopSlot slot) {
        super(baseName, isConstant);
        m_motor = motor;
        m_parentConfig = parentConfig;
        m_closedLoopConfig = new ClosedLoopConfig();
        m_slot = slot;
    }

    @Override
    public IPidPropertyBuilder addP(double defaultValue) {
        addP(defaultValue, (double gain) -> {
            m_closedLoopConfig.p(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addI(double defaultValue) {
        addI(defaultValue, (double gain) -> {
            m_closedLoopConfig.i(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addD(double defaultValue) {
        addD(defaultValue, (double gain) -> {
            m_closedLoopConfig.d(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addFF(double defaultValue) {
        addFF(defaultValue, (double gain) -> {
            m_closedLoopConfig.velocityFF(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addMaxVelocity(double defaultValue) {
        addMaxVelocity(defaultValue, (double gain) -> {
            m_closedLoopConfig.maxMotion.maxVelocity(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    @Override
    public IPidPropertyBuilder addMaxAcceleration(double defaultValue) {
        addMaxAcceleration(defaultValue, (double gain) -> {
            m_closedLoopConfig.maxMotion.maxAcceleration(gain, m_slot);
            configureMotor();
        });
        return this;
    }

    private void configureMotor() {
        SparkBaseConfig config;
        if (m_motor instanceof SparkMax) {
            config = new SparkMaxConfig().apply(m_closedLoopConfig);
        }
        else if (m_motor instanceof SparkFlex) {
            config = new SparkFlexConfig().apply(m_closedLoopConfig);
        } else {
            throw new IllegalArgumentException("Unexpected motor type");
        }
        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public PidProperty build() {
        PidProperty output = super.build();
        m_parentConfig.apply(m_closedLoopConfig);
        return output;
    }
}
