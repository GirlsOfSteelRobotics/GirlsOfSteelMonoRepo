package com.gos.chargedup.subsystems;


import com.gos.chargedup.Constants;
import com.gos.chargedup.commands.CheckRumbleCommand;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.properties.GosIntProperty;
import com.gos.lib.properties.HeavyIntegerProperty;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.gos.lib.rev.checklists.SparkMaxMotorsMoveChecklist;
import com.revrobotics.RelativeEncoder;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ClawSubsystem extends SubsystemBase {
    private static final GosDoubleProperty HOLD_SPEED = new GosDoubleProperty(true, "ClawHoldSpeed", 0.2);
    private static final GosDoubleProperty CLAW_IN_SPEED = new GosDoubleProperty(true, "ClawInSpeed", 0.8);
    private static final GosDoubleProperty CLAW_OUT_SPEED = new GosDoubleProperty(true, "ClawOutSpeed", 1.0);
    private static final GosIntProperty CLAW_CURRENT_LIMIT = new GosIntProperty(true, "ClawCurrentLimit", 25);
    private static final GosDoubleProperty POSSESSION_OF_PIECE_CURRENT = new GosDoubleProperty(true, "ClawCheckHasPieceCurrent", 5);
    private static final GosDoubleProperty POSSESSION_OF_PIECE_CURRENT_VELOCITY = new GosDoubleProperty(true, "ClawCheckHasPieceVelocity", 1);

    private static final double AUTO_EJECTION_TIME = 0.5;
    private static final double AUTO_INTAKE_TIME = 0.5;

    private final SparkMax m_clawMotor;
    private final RelativeEncoder m_clawEncoder;
    private final SparkMaxAlerts m_clawMotorErrorAlerts;
    private final HeavyIntegerProperty m_currentLimit;

    private final LoggingUtil m_networkTableEntries;

    public ClawSubsystem() {
        m_clawMotor = new SparkMax(Constants.CLAW_MOTOR, MotorType.kBrushless);
        SparkMaxConfig clawMotorConfig = new SparkMaxConfig();

        m_clawMotor.setInverted(true);
        m_clawEncoder = m_clawMotor.getEncoder();
        clawMotorConfig.idleMode(IdleMode.kBrake);
        clawMotorConfig.smartCurrentLimit(10);
        m_clawMotor.configure(clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_clawMotorErrorAlerts = new SparkMaxAlerts(m_clawMotor, "claw motor");

        currentLimit = new HeavyIntegerProperty(m_clawMotor:Config.smartCurrentLimit, CLAW_CURRENT_LIMIT);

        m_networkTableEntries = new LoggingUtil("Claw Subsystem");
        m_networkTableEntries.addDouble("Current Amps", m_clawMotor::getOutputCurrent);
        m_networkTableEntries.addDouble("Output", m_clawMotor::getAppliedOutput);
        m_networkTableEntries.addDouble("Velocity (RPM)", m_clawEncoder::getVelocity);
    }

    //intake close
    public void moveClawIntakeIn() {
        m_clawMotor.set(CLAW_IN_SPEED.getValue());
    }

    public void holdPiece() {
        m_clawMotor.set(HOLD_SPEED.getValue());
    }

    //intake open
    public void moveClawIntakeOut() {
        m_clawMotor.set(-CLAW_OUT_SPEED.getValue());
    }

    public void stopIntake() {
        m_clawMotor.set(0);
    }

    public boolean hasGamePiece() {
        return (m_clawMotor.getOutputCurrent() > POSSESSION_OF_PIECE_CURRENT.getValue()
            && (Math.abs(m_clawEncoder.getVelocity()) < POSSESSION_OF_PIECE_CURRENT_VELOCITY.getValue()));

    }

    @Override
    public void periodic() {
        m_currentLimit.updateIfChanged();
        m_clawMotorErrorAlerts.checkAlerts();
    }

    public void clearStickyFaults() {
        m_clawMotor.clearFaults();
    }

    //////////////
    // Checklists
    //////////////
    public Command createIsClawMotorMovingChecklist() {
        return new SparkMaxMotorsMoveChecklist(this, m_clawMotor, "Claw: Motor", 1.0);
    }

    /////////////////////
    // Command Factories
    /////////////////////
    public Command createMoveClawIntakeInCommand() {
        return this.runEnd(this::moveClawIntakeIn, this::stopIntake).withName("ClawIntakeIn");
    }

    public Command createMoveClawIntakeInNoStopCommand() {
        return this.run(this::moveClawIntakeIn);
    }

    public Command createMoveClawIntakeInWithTimeoutCommand() {
        return createMoveClawIntakeInCommand().withTimeout(AUTO_INTAKE_TIME);
    }

    public Command createMoveClawIntakeOutCommand() {
        return this.runEnd(this::moveClawIntakeOut, this::stopIntake).withName("ClawIntakeOut");
    }

    public Command createMoveClawIntakeOutWithTimeoutCommand() {
        return createMoveClawIntakeOutCommand().withTimeout(AUTO_EJECTION_TIME);
    }

    public Command createTeleopMoveClawIntakeInCommand(CommandXboxController joystick) {
        return this.createMoveClawIntakeInCommand().alongWith(new CheckRumbleCommand(joystick, this::hasGamePiece));
    }

    public Command createHoldPieceCommand() {
        return this.run(this::holdPiece);
    }
}
