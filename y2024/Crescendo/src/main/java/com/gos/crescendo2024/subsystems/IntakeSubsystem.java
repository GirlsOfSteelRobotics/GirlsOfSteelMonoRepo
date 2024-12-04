package com.gos.crescendo2024.subsystems;


import com.gos.crescendo2024.Constants;
import com.gos.lib.logging.LoggingUtil;
import com.gos.lib.properties.GosDoubleProperty;
import com.gos.lib.rev.alerts.SparkMaxAlerts;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static final GosDoubleProperty INTAKE_OUT_SPEED = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "Intake_Out_Speed", -1);
    private static final GosDoubleProperty INTAKE_IN_SPEED = new GosDoubleProperty(Constants.DEFAULT_CONSTANT_PROPERTIES, "Intake_In_Speed", 1.0);

    private final SparkMax m_intakeMotor;
    private final RelativeEncoder m_intakeEncoder;
    private final DigitalInput m_photoelectricSensor;
    private final LoggingUtil m_networkTableEntries;
    private final SparkMaxAlerts m_intakeAlert;

    public IntakeSubsystem() {
        m_intakeMotor = new SparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(40);
        m_intakeMotor.setInverted(true);
        m_intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakeAlert = new SparkMaxAlerts(m_intakeMotor, "Intake Motor");

        m_photoelectricSensor = new DigitalInput(Constants.INTAKE_SENSOR);

        m_networkTableEntries = new LoggingUtil("Intake Subsystem");
        m_networkTableEntries.addDouble("Current Velocity", m_intakeEncoder::getVelocity);
        m_networkTableEntries.addDouble("Current (Amps)", m_intakeMotor::getOutputCurrent);
        m_networkTableEntries.addBoolean("Has Piece", this::hasGamePiece);
    }

    @Override
    public void periodic() {
        m_networkTableEntries.updateLogs();
        m_intakeAlert.checkAlerts();

    }

    public void intakeIn() {
        m_intakeMotor.set(INTAKE_IN_SPEED.getValue());
    }

    public void intakeOut() {
        m_intakeMotor.set(INTAKE_OUT_SPEED.getValue());
    }

    public void intakeStop() {
        m_intakeMotor.set(0);
    }

    public boolean hasGamePiece() {
        return !m_photoelectricSensor.get();
    }

    public double getIntakeMotorPercentage() {
        return m_intakeMotor.getAppliedOutput();
    }

    public void clearStickyFaults() {
        m_intakeMotor.clearFaults();
    }

    private void setIdleMode(IdleMode idleMode) {
        intakeMotorConfig.idleMode(idleMode);
    }

    /////////////////////////////////////
    // Command Factories
    /////////////////////////////////////
    public Command createMoveIntakeInCommand() {
        return this.runEnd(this::intakeIn, this::intakeStop).withName("Intake In");
    }

    public Command createMoveIntakeInWithTimeoutCommand() {
        return createMoveIntakeInCommand().withTimeout(1)
            .andThen(createMoveIntakeOutCommand()).withTimeout(0.1)
            .withName("Intake In With Timeout");
    }

    public Command createMoveIntakeOutCommand() {
        return this.runEnd(this::intakeOut, this::intakeStop).withName("Intake Out");
    }

    public Command createIntakeUntilPieceCommand() {
        return createMoveIntakeInCommand().until(this::hasGamePiece).withName("Intake Till Piece");
    }

    public Command createIntakeToCoastCommand() {
        return this.runEnd(
                () -> setIdleMode(IdleMode.kCoast),
                () -> setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true).withName("Intake to Coast");
    }
}
