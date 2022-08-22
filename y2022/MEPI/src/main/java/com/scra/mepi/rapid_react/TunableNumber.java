package com.scra.mepi.rapid_react;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber {
    private static final String TABLE_KEY = "TunableNumbers";

    private final String m_key;
    private double m_defaultValue;

    private double m_lastHasChangedValue = m_defaultValue;

    /**
     * Create a new TunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public TunableNumber(String dashboardKey) {
        this.m_key = TABLE_KEY + "/" + dashboardKey;
    }

    /**
     * Create a new TunableNumber wit1h the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        setDefault(defaultValue);
    }

    /**
     * Get the default value for the number that has been set
     *
     * @return The default value
     */
    public double getDefault() {
        return m_defaultValue;
    }

    /**
     * Set the default value of the number
     *
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue) {
        this.m_defaultValue = defaultValue;
        SmartDashboard.putNumber(m_key, SmartDashboard.getNumber(m_key, defaultValue));
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     *
     * @return The current value
     */
    public double get() {
        // System.out.println("place 1");
        return SmartDashboard.getNumber(m_key, m_defaultValue);
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @return True if the number has changed since the last time this method was called, false
     * otherwise
     */
    public boolean hasChanged() {
        // System.out.println("place 2");
        double currentValue = get();
        if (currentValue != m_lastHasChangedValue) {
            m_lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }
}
