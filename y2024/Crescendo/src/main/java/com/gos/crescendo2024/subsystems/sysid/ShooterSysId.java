package com.gos.crescendo2024.subsystems.sysid;

import com.gos.crescendo2024.subsystems.ShooterSubsystem;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSysId {
    private final SysIdRoutine m_routine;
    private final ShooterSubsystem m_shooter;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_rotations = Rotations.mutable(0);
    private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

    public ShooterSysId(ShooterSubsystem shooter) {
        m_shooter = shooter;
        m_routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(7),
                Seconds.of(10)),
            new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, shooter)
        );
    }

    private void setVoltage(Voltage volts) {
        m_shooter.setVoltage(volts.in(Volts));
    }

    private void logMotors(SysIdRoutineLog log) {
        log.motor("shooter")
            .voltage(
                m_appliedVoltage.mut_replace(
                    m_shooter.getVoltage(), Volts))
            .angularPosition(m_rotations.mut_replace(m_shooter.getEncoderPos(), Rotations))
            .angularVelocity(
                m_velocity.mut_replace(m_shooter.getRPM(), RotationsPerSecond));
    }

    ///////////////////////
    // Command Factories
    ///////////////////////
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routine.dynamic(direction);
    }
}
