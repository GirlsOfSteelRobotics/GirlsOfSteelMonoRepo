package com.gos.crescendo2024.subsystems;

import com.gos.crescendo2024.Constants;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.SparkMaxUtil;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangerSubsystem extends SubsystemBase {
    private static final GosDoubleProperty HANGER_DOWN_SPEED = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "Hanger_Down_Speed", -0.3);
    private static final GosDoubleProperty HANGER_UP_SPEED = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "Hanger_Up_Speed", 0.3);

    private static final GosDoubleProperty HANGER_UP_GOAL_POSITION = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "Hanger Up Goal Position", 118);

    private final SparkMax m_leftHangerMotor;
    private final RelativeEncoder m_leftHangerEncoder;
    private final SparkMaxAlerts m_leftHangerAlert;
    private final SparkClosedLoopController m_leftPidController;
    private final PidProperty m_leftPidProperties;

    private final SparkMax m_rightHangerMotor;
    private final RelativeEncoder m_rightHangerEncoder;
    private final SparkMaxAlerts m_rightHangerAlert;
    private final SparkClosedLoopController m_rightPidController;
    private final PidProperty m_rightPidProperties;

    private final LoggingUtil m_networkTableEntries;
    private final DigitalInput m_upperLimitSwitchLeft;
    private final DigitalInput m_lowerLimitSwitchLeft;

    private final DigitalInput m_upperLimitSwitchRight;
    private final DigitalInput m_lowerLimitSwitchRight;

    //TODO add limit switches

    public HangerSubsystem() {
        m_leftHangerMotor = new SparkMax(Constants.HANGER_LEFT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftHangerMotorConfig = new SparkMaxConfig();
        m_leftHangerMotor.setInverted(true);
        m_leftHangerEncoder = m_leftHangerMotor.getEncoder();
        leftHangerMotorConfig.idleMode(IdleMode.kBrake);
        leftHangerMotorConfig.smartCurrentLimit(60);
        m_leftPidController = m_leftHangerMotor.getClosedLoopController();
        m_leftPidProperties = createPidProperties(m_leftPidController);
        m_leftHangerMotor.configure(leftHangerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftHangerAlert = new SparkMaxAlerts(m_leftHangerMotor, "hanger a");

        m_rightHangerMotor = new SparkMax(Constants.HANGER_RIGHT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig rightHangerMotorConfig = new SparkMaxConfig();
        m_rightHangerMotor.setInverted(false);
        m_rightHangerEncoder = m_rightHangerMotor.getEncoder();
        rightHangerMotorConfig.idleMode(IdleMode.kBrake);
        rightHangerMotorConfig.smartCurrentLimit(60);
        m_rightPidController = m_rightHangerMotor.getClosedLoopController();
        m_rightPidProperties = createPidProperties(m_rightPidController);
        m_rightHangerMotor.configure(rightHangerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightHangerAlert = new SparkMaxAlerts(m_rightHangerMotor, "hanger b");


        m_upperLimitSwitchLeft = new DigitalInput(Constants.HANGER_UPPER_LIMIT_SWITCH_LEFT);
        m_lowerLimitSwitchLeft = new DigitalInput(Constants.HANGER_LOWER_LIMIT_SWITCH_LEFT);
        m_upperLimitSwitchRight = new DigitalInput(Constants.HANGER_UPPER_LIMIT_SWITCH_RIGHT);
        m_lowerLimitSwitchRight = new DigitalInput(Constants.HANGER_LOWER_LIMIT_SWITCH_RIGHT);

        m_networkTableEntries = new LoggingUtil("Hanger Subsystem");
        m_networkTableEntries.addDouble("Left Output: ", m_leftHangerMotor::getAppliedOutput);
        m_networkTableEntries.addDouble("Right Output", m_rightHangerMotor::getAppliedOutput);
        m_networkTableEntries.addDouble("Left Pos: ", m_leftHangerEncoder::getPosition);
        m_networkTableEntries.addDouble("Right Pos", m_rightHangerEncoder::getPosition);

        resetEncoders();

    }

    private PidProperty createPidProperties(SparkClosedLoopController pidController) {
        return new RevPidPropertyBuilder("HangerPid", false, pidController, 0)
            .addP(0.1)
            .build();
    }

    public void moveHangerToPosition(double position) {
        m_leftPidController.setReference(position, ControlType.kPosition);
        m_rightPidController.setReference(position, ControlType.kPosition);
    }


    @Override
    public void periodic() {
        m_leftHangerAlert.checkAlerts();
        m_rightHangerAlert.checkAlerts();
        m_networkTableEntries.updateLogs();

        m_leftPidProperties.updateIfChanged();
        m_rightPidProperties.updateIfChanged();
    }

    public void clearStickyFaults() {
        m_leftHangerMotor.clearFaults();
        m_rightHangerMotor.clearFaults();
    }

    private void setLeftMotor(double speed) {
        m_leftHangerMotor.set(speed);
    }

    private void setRightMotor(double speed) {
        m_rightHangerMotor.set(speed);
    }

    public void runHangerUp() {
        runLeftHangerUp();
        runRightHangerUp();
    }

    public void runLeftHangerUp() {
        if (isLeftUpperLimitSwitchedPressed()) {
            m_leftHangerMotor.set(0);
        } else {
            setLeftMotor(HANGER_UP_SPEED.getValue());
        }
    }

    public void runRightHangerUp() {
        if (isRightUpperLimitSwitchedPressed()) {
            m_rightHangerMotor.set(0);
        } else {
            setRightMotor(HANGER_UP_SPEED.getValue());
        }
    }

    public void runHangerDown() {
        runLeftHangerDown();
        runRightHangerDown();
    }

    public void runLeftHangerDown() {
        if (isLeftLowerLimitSwitchedPressed()) {
            m_leftHangerMotor.set(0);
        } else {
            setLeftMotor(HANGER_DOWN_SPEED.getValue());
        }
    }

    public void runRightHangerDown() {
        if (isRightLowerLimitSwitchedPressed()) {
            m_rightHangerMotor.set(0);
        } else {
            setRightMotor(HANGER_DOWN_SPEED.getValue());
        }
    }

    public void stopHanger() {
        m_leftHangerMotor.set(0);
        m_rightHangerMotor.set(0);
    }

    private void resetEncoders() {
        SparkMaxUtil.autoRetry(() -> m_leftHangerEncoder.setPosition(0));
        SparkMaxUtil.autoRetry(() -> m_rightHangerEncoder.setPosition(0));
    }

    public boolean isRightUpperLimitSwitchedPressed() {
        return !m_upperLimitSwitchRight.get();
    }

    public boolean isRightLowerLimitSwitchedPressed() {
        return !m_lowerLimitSwitchRight.get();
    }

    public boolean isLeftLowerLimitSwitchedPressed() {
        return !m_lowerLimitSwitchLeft.get();
    }

    public boolean isLeftUpperLimitSwitchedPressed() {
        return !m_upperLimitSwitchLeft.get();
    }

    public void setIdleMode(IdleMode mode) {
        m_leftHangerMotor.setIdleMode(mode);
        m_rightHangerMotor.setIdleMode(mode);
    }

    /////////////////////////////////////
    // Command Factories
    /////////////////////////////////////
    public Command createHangerUp() {
        return this.runEnd(this::runHangerUp, this::stopHanger).withName("Hanger Up");
    }

    public Command createHangerDown() {
        return this.runEnd(this::runHangerDown, this::stopHanger).withName("Hanger Down");
    }

    public Command createLeftHangerUp() {
        return runEnd(this::runLeftHangerUp, this::stopHanger).withName("Left Hanger Up");
    }

    public Command createLeftHangerDown() {
        return runEnd(this::runLeftHangerDown, this::stopHanger).withName("Left Hanger Down");
    }

    public Command createRightHangerUp() {
        return runEnd(this::runRightHangerUp, this::stopHanger).withName("Right Hanger Up");
    }

    public Command createRightHangerDown() {
        return runEnd(this::runRightHangerDown, this::stopHanger).withName("Right Hanger Down");
    }

    public Command createSetHangerToCoast() {
        return this.runEnd(
                () -> setIdleMode(IdleMode.kCoast),
                () -> setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true).withName("Hangers to Coast");
    }

    public Command createAutoUpCommand() {
        return runEnd(() -> moveHangerToPosition(HANGER_UP_GOAL_POSITION.getValue()), this::stopHanger)
            .withName("AutoHangerUp");
    }

    public Command createAutoDownCommand() {
        return runEnd(() -> moveHangerToPosition(0), this::stopHanger)
            .withName("AutoHangerDown");
    }

    public Command createResetEncoders() {
        return run(this::resetEncoders)
            .ignoringDisable(true)
            .withName("Reset Encoders");
    }
}
