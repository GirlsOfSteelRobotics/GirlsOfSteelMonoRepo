package com.gos.codelabs.basic_simulator.commands;

import com.gos.codelabs.basic_simulator.subsystems.ElevatorSubsystem;
import com.gos.codelabs.basic_simulator.BaseTestFixture;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class ElevatorToPositionCommandTest extends BaseTestFixture {

    @Test
    public void testExecution() {
        ElevatorSubsystem.Positions goal = ElevatorSubsystem.Positions.MID;
        try (ElevatorSubsystem elevator = new ElevatorSubsystem()) {
            ElevatorToPositionCommand command = new ElevatorToPositionCommand(elevator, goal);
            command.schedule();

            runCycles(100, null, () -> !command.isScheduled());

            assertFalse(command.isScheduled());
            assertEquals(goal.m_heightMeters, elevator.getHeight(), ElevatorSubsystem.ALLOWABLE_POSITION_ERROR);
        }
    }
}
