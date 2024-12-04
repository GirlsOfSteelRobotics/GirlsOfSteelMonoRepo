// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.scra.mepi.rapid_react.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.scra.mepi.rapid_react.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax m_leftClimber = new SparkMax(Constants.CLIMBER_LEFT, MotorType.kBrushless);
    private final SparkMax m_rightClimber =
        new SparkMax(Constants.CLIMBER_RIGHT, MotorType.kBrushless);
    private final DigitalInput m_leftLimitSwitch = new DigitalInput(Constants.LEFT_LIMIT_SWITCH);
    private final DigitalInput m_rightLimitSwitch = new DigitalInput(Constants.RIGHT_LIMIT_SWITCH);
    private final RelativeEncoder m_leftEncoder = m_leftClimber.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightClimber.getEncoder();
    private final ProfiledPIDController m_climberPID = new ProfiledPIDController(
        3.5, 0.0, 0.0, new Constraints(20.0, 150.0), 0.02);

    /**
     * Creates a new ClimberSubsystem.
     */
    public ClimberSubsystem() {
        SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
        SparkMaxConfig rightClimberConfig = new SparkMaxConfig();
        m_leftClimber.setInverted(true);
        m_rightClimber.setInverted(false);

        rightClimberConfig.follow(m_leftClimber, true);
        leftClimberConfig.smartCurrentLimit(50);
        rightClimberConfig.smartCurrentLimit(50);

        m_leftClimber.configure(leftClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightClimber.configure(rightClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runClimberPID(double goal) {
        m_climberPID.setGoal(goal);
        set(m_climberPID.calculate(m_leftEncoder.getPosition()));
    }


    @SuppressWarnings("PMD.AvoidReassigningParameters")
    public void set(double speed) {
        if (speed < 0 && (leftLimitSwitchPress() || rightLimitSwitchPress())) {
            speed = 0;
        }
        if (speed > 0 && m_leftEncoder.getPosition() > 150) {
            speed = 0;
        }
        m_leftClimber.set(speed);
    }

    public boolean leftLimitSwitchPress() {
        return m_leftLimitSwitch.get();
    }

    public boolean rightLimitSwitchPress() {
        return !m_rightLimitSwitch.get();
    }

    public double getPIDPosition() {
        return m_leftEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (leftLimitSwitchPress() || rightLimitSwitchPress()) {
            m_leftEncoder.setPosition(0);
            m_rightEncoder.setPosition(0);
        }

        SmartDashboard.putBoolean("left limit", leftLimitSwitchPress());
        SmartDashboard.putBoolean("right limit", rightLimitSwitchPress());
        SmartDashboard.putNumber("climb left encoder", m_leftEncoder.getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
