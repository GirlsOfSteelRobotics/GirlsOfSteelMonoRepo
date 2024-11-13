package com.gos.lib.rev.alerts;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;

@SuppressWarnings({"PMD.CyclomaticComplexity", "PMD.NPathComplexity"})
public class SparkMaxAlerts {
    public final SparkBase m_sparkMax;
    public final String m_motorString;
    public final Alert m_faultAlert;
    public final Alert m_stickyFaultAlert;
    public final Alert m_warningsAlert;
    public final Alert m_stickyWarningAlert;

    public SparkMaxAlerts(SparkBase sparkMax, String motor) {
        m_sparkMax = sparkMax;
        m_motorString = motor;

        m_faultAlert = new Alert(m_motorString, Alert.AlertType.ERROR);
        m_stickyFaultAlert = new Alert(m_motorString, Alert.AlertType.WARNING);

        m_warningsAlert = new Alert(m_motorString, AlertType.WARNING);
        m_stickyWarningAlert = new Alert(m_motorString, AlertType.INFO);
    }

    public void checkAlerts() {
        checkFaults();
        checkStickyFaults();
        checkWarnings();
        checkStickyWarnings();
    }

    private void checkFaults() {
        String errorString = getFaultString(m_sparkMax.hasActiveFault(), m_sparkMax.getFaults());
        m_faultAlert.setText(errorString);
        m_faultAlert.set(!(errorString.equals(m_motorString)));
    }

    private void checkStickyFaults() {
        String errorString = getFaultString(m_sparkMax.hasStickyFault(), m_sparkMax.getStickyFaults());
        m_stickyFaultAlert.setText(errorString);
        m_stickyFaultAlert.set(!(errorString.equals(m_motorString)));
    }

    private void checkWarnings() {
        String errorString = getWarningString(m_sparkMax.hasActiveWarning(), m_sparkMax.getWarnings());
        m_warningsAlert.setText(errorString);
        m_warningsAlert.set(!(errorString.equals(m_motorString)));
    }

    private void checkStickyWarnings() {
        String errorString = getWarningString(m_sparkMax.hasStickyWarning(), m_sparkMax.getStickyWarnings());
        m_stickyWarningAlert.setText(errorString);
        m_stickyWarningAlert.set(!(errorString.equals(m_motorString)));
    }

    private String getFaultString(boolean hasFault, Faults faults) {
        StringBuilder errorBuilder = new StringBuilder(m_motorString);
        if (!hasFault) {
            return errorBuilder.toString();
        }

        errorBuilder.append("Faults Detected: ");

        if (faults.other) {
            errorBuilder.append("Other");
        }
        if (faults.motorType) {
            errorBuilder.append("Motor Type");
        }
        if (faults.sensor) {
            errorBuilder.append("Sensor");
        }
        if (faults.can) {
            errorBuilder.append("CAN");
        }
        if (faults.temperature) {
            errorBuilder.append("Temperature");
        }
        if (faults.gateDriver) {
            errorBuilder.append("Gate Driver");
        }
        if (faults.escEeprom) {
            errorBuilder.append("EEPROM");
        }
        if (faults.firmware) {
            errorBuilder.append("Firmware");
        }

        return errorBuilder.toString();
    }

    private String getWarningString(boolean hasWarning, Warnings warnings) {
        StringBuilder errorBuilder = new StringBuilder(m_motorString);
        if (!hasWarning) {
            return errorBuilder.toString();
        }

        errorBuilder.append("Warnings Detected: ");

        if (warnings.brownout) {
            errorBuilder.append("Brownout");
        }
        if (warnings.overcurrent) {
            errorBuilder.append("Overcurrent");
        }
        if (warnings.escEeprom) {
            errorBuilder.append("ESC EEPROM");
        }
        if (warnings.extEeprom) {
            errorBuilder.append("EXT EEPROM");
        }
        if (warnings.sensor) {
            errorBuilder.append("Sensor");
        }
        if (warnings.stall) {
            errorBuilder.append("Stall");
        }
        if (warnings.hasReset) {
            errorBuilder.append("Has Reset");
        }
        if (warnings.other) {
            errorBuilder.append("Other");
        }

        return errorBuilder.toString();
    }
}
