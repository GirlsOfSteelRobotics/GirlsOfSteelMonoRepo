package com.gos.lib.rev.checklists;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.frc2023.util.Alert;

public class SparkMaxMotorsMoveChecklist extends Command {

    private final RelativeEncoder m_encoder;

    private final CANSparkMax m_motor;

    private final Timer m_timer;

    private double m_startPosMotor;

    private final String m_label;

    private final double m_expectedDist;

    private final Alert m_alert;

    public SparkMaxMotorsMoveChecklist(Subsystem subsystem, CANSparkMax motor, String label, double expectedDist) {
        addRequirements(subsystem);
        m_encoder = motor.getEncoder();
        m_motor = motor;
        m_startPosMotor = 0;
        m_timer = new Timer();
        m_label = label;
        m_expectedDist = expectedDist;
        m_alert = new Alert(m_label, Alert.AlertType.ERROR);

    }

    @Override
    public void initialize() {
        m_motor.set(0);
        m_startPosMotor = m_encoder.getPosition();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_motor.set(0.25);
    }

    @Override
    public boolean isFinished() {
        return (m_timer.get() >= 1);
    }

    @Override
    public void end(boolean interrupted) {
        m_motor.set(0);
        boolean isRobotAtPos = m_encoder.getPosition() - m_startPosMotor >= m_expectedDist;
        m_alert.set(!isRobotAtPos);
        m_timer.stop();
    }
}
