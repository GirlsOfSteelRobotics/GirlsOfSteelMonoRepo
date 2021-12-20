package girlsofsteel.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import girlsofsteel.subsystems.Turret;

public class TESTSetPointTurret extends CommandBase {

    private final Turret m_turret;
    private double m_angle;

    public TESTSetPointTurret(Turret turret) {
        m_turret = turret;
        requires(m_turret);
        SmartDashboard.putNumber("Turret Relative Angle", 0.0);
    }

    @Override
    protected void initialize() {
        m_turret.initEncoder();
        m_turret.enablePID();
    }

    @Override
    protected void execute() {
        m_angle = SmartDashboard.getNumber("Turret Relative Angle", 0.0);
        m_turret.setPIDSetPoint(m_turret.getEncoderDistance() + m_angle);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        m_turret.stopJag();
    }

    @Override
    protected void interrupted() {
        end();
    }
}
