package com.gos.crescendo2024.subsystems.sysid;

import com.gos.crescendo2024.subsystems.ArmPivotSubsystem;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.measure.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ArmPivotSysId {
    private final SysIdRoutine m_routine;
    private final ArmPivotSubsystem m_armPivot;

    private final MutableVoltage m_appliedVoltage = mutable(Volts.of(0));
    private final MutableAngle m_angle = mutable(Degrees.of(0));
    private final MutableAngularVelocity m_velocity = mutable(DegreesPerSecond.of(0));

    public ArmPivotSysId(ArmPivotSubsystem armPivot) {
        m_armPivot = armPivot;
        m_routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(.25).per(Units.Seconds.of(1.0)),
                Units.Volts.of(2.5),
                Units.Seconds.of(10.0)),
            new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, armPivot)
        );
    }

    private void setVoltage(Voltage volts) {
        m_armPivot.setVoltage(volts.in(Volts));
    }

    private void logMotors(SysIdRoutineLog log) {
        log.motor("armPivot")
            .voltage(
                m_appliedVoltage.mut_replace(
                    m_armPivot.getVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(m_armPivot.getAngle(), Degrees))
            .angularVelocity(
                m_velocity.mut_replace(m_armPivot.getEncoderVel(), DegreesPerSecond));
    }

    private boolean getAngleLimit(SysIdRoutine.Direction direction) {
        if (direction == SysIdRoutine.Direction.kForward) {
            return m_armPivot.getAngle() >= 75;
        }
        else {
            return m_armPivot.getAngle() <= 15;
        }

    }

    ///////////////////////
    // Command Factories
    ///////////////////////
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routine.quasistatic(direction)
            .until(() -> getAngleLimit(direction))
            .finallyDo(m_armPivot::stopArmMotor);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routine.dynamic(direction)
            .until(() -> getAngleLimit(direction))
            .finallyDo(m_armPivot::stopArmMotor);
    }
}
