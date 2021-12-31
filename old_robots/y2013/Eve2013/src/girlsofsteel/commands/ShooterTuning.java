/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 *
 * Make shoot method that given a desired encoder speed and current battery voltage,
 * it will set the jags to a suitable speed from the table. Also be prepared to
 * extrapolate values (see code from last year).
 */

package girlsofsteel.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import girlsofsteel.subsystems.Shooter;

/**
 * @author Sylvie
 */
public class ShooterTuning extends CommandBase {

    private final Shooter m_shooter;
    private double m_batteryVoltage;
    private int m_counter;
    private boolean m_done;
    private double m_speed;
    private double m_time;

    public ShooterTuning(Shooter shooter) {
        m_shooter = shooter;
        requires(m_shooter);
    }

    @Override
    protected void initialize() {
        SmartDashboard.putNumber("speed", 0.0);
        SmartDashboard.putBoolean("test speed", false);
    }

    @Override
    protected void execute() {
        if (SmartDashboard.getBoolean("test speed", false)) {
            m_batteryVoltage = RobotController.getBatteryVoltage();
            m_speed = SmartDashboard.getNumber("speed", 0.0);
            m_time = timeSinceInitialized();
            m_shooter.setJags(m_speed);
            if (timeSinceInitialized() - m_time > 3) {
                m_shooter.fillArray(m_speed, m_shooter.getEncoderRate(), m_batteryVoltage);
                m_time = timeSinceInitialized();
                m_counter++;
            }
            //arbitrary number
            if (m_counter == 25) {
                m_done = true;
            }
        }
        System.out.println("Point Voltage: " + m_speed + "\t");
        System.out.print("Point Shooter Encoder Speed: " + m_shooter.getEncoderRate());
        System.out.println("Point Battery Voltage: " + m_batteryVoltage);

    }

    @Override
    protected boolean isFinished() {
        return m_done;
    }

    @Override
    protected void end() {
        m_shooter.stopJags();
    }

    @Override
    protected void interrupted() {
        end();
    }
}
