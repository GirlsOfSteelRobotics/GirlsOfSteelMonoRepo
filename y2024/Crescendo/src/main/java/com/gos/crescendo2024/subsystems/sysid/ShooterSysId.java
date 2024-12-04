package com.gos.crescendo2024.subsystems.sysid;

import com.gos.crescendo2024.subsystems.ShooterSubsystem;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.measure.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSysId {
    private final SysIdRoutine m_routine;
    private final ShooterSubsystem m_shooter;

    private final MutableVoltage m_appliedVoltage = mutable(Volts.of(0));
    private final MutableAngle m_rotations = mutable(Rotations.of(0));
    private final MutableAngularVelocity m_velocity = mutable(RotationsPerSecond.of(0));

    public ShooterSysId(ShooterSubsystem shooter) {
        m_shooter = shooter;
        m_routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1.0).per(Units.Seconds.of(1.0)),
                Units.Volts.of(7.0),
                Units.Seconds.of(10.0)),
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
