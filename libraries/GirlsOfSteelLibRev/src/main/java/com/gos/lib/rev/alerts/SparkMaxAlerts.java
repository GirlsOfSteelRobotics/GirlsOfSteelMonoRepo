package com.gos.lib.rev.alerts;

import com.revrobotics.spark.SparkBase;
import org.littletonrobotics.frc2023.util.Alert;

public class SparkMaxAlerts {
    public final SparkBase m_sparkMax;
    public final String m_motorString;
    public final Alert m_alert;
    public final Alert m_alertSticky;

    public SparkMaxAlerts(SparkBase sparkMax, String motor) {
        m_sparkMax = sparkMax;
        m_motorString = motor;
        m_alert = new Alert(m_motorString, Alert.AlertType.ERROR);
        m_alertSticky = new Alert(m_motorString, Alert.AlertType.WARNING);
    }

    public void checkAlerts() {
        checkFaults();
        checkStickyFaults();
    }

    private void checkFaults() {
        short bitmask = m_sparkMax.getFaults();

        StringBuilder errorBuilder = new StringBuilder(m_motorString);
        for (SparkBase.FaultID faultId : SparkBase.FaultID.values()) {
            if ((bitmask & (1 << faultId.value)) != 0) {
                errorBuilder.append('\n').append(faultId);
            }
        }

        String errorString = errorBuilder.toString();
        m_alert.setText(errorString);

        m_alert.set(!(errorString.equals(m_motorString)));
    }

    private void checkStickyFaults() {
        short bitmask = m_sparkMax.getStickyFaults();

        StringBuilder errorBuilder = new StringBuilder(m_motorString);
        for (SparkBase.FaultID faultId : SparkBase.FaultID.values()) {
            if ((bitmask & (1 << faultId.value)) != 0) {
                errorBuilder.append('\n').append(faultId);
            }
        }

        String errorString = errorBuilder.toString();
        m_alertSticky.setText(errorString);

        m_alertSticky.set(!(errorString.equals(m_motorString)));


    }
}
