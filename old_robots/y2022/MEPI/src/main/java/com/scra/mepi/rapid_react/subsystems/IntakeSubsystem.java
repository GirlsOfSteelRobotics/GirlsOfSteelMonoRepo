// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.scra.mepi.rapid_react.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(6, MotorType.kBrushless);
    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 6);

    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        m_motor.setInverted(true);
        motorConfig.smartCurrentLimit(50);
        retract();

        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double speed) {
        m_motor.set(speed);
        if (speed == 0) {
            retract();
        } else {
            extend();
        }
    }

    public void extend() {
        m_solenoid.set(Value.kForward);
    }

    public final void retract() {
        m_solenoid.set(Value.kReverse);
    }

    public void toggle() {
        m_solenoid.toggle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
