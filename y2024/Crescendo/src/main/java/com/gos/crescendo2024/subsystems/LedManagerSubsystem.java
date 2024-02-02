package com.gos.crescendo2024.subsystems;


import com.gos.crescendo2024.Constants;
import com.gos.crescendo2024.led_patterns.AutoPattern;
import com.gos.crescendo2024.led_patterns.HasPiecePattern;
import com.gos.crescendo2024.led_patterns.TeleopPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedManagerSubsystem extends SubsystemBase {

    private static final int MAX_INDEX_LED = 30;


    // Led core
    protected final AddressableLEDBuffer m_buffer;
    protected final AddressableLED m_led;

    private final TeleopPattern m_teleopPattern;
    private final HasPiecePattern m_hasPiecePattern;
    private final AutoPattern m_autoPattern;
    private final IntakeSubsystem m_intakeSubsystem;



    public LedManagerSubsystem(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_buffer = new AddressableLEDBuffer(MAX_INDEX_LED);
        m_led = new AddressableLED(Constants.LED_PORT);

        m_led.setLength(m_buffer.getLength());

        // Set the data
        m_led.setData(m_buffer);
        m_led.start();

        m_teleopPattern = new TeleopPattern(MAX_INDEX_LED, m_buffer);
        m_hasPiecePattern = new HasPiecePattern(MAX_INDEX_LED, m_buffer);
        m_autoPattern = new AutoPattern(MAX_INDEX_LED, m_buffer);

    }

    @Override
    public void periodic() {
        clear();

        if (DriverStation.isTeleop()) {
            m_teleopPattern.writeLED();
        }
        if (m_intakeSubsystem.hasGamePiece()) {
            m_hasPiecePattern.writeHasPiecePattern();
        }
        if (DriverStation.isAutonomous()) {
            m_autoPattern.writeAutoPattern();
        }
        // driverPracticePatterns();
        m_led.setData(m_buffer);
    }

    private void clear() {
        for (int i = 0; i < MAX_INDEX_LED; i++) {
            m_buffer.setRGB(i, 0, 0, 0);
        }
    }
}

