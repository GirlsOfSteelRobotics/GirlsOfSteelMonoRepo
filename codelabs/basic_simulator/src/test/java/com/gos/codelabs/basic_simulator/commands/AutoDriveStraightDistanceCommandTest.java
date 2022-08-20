package com.gos.codelabs.basic_simulator.commands;

import com.gos.codelabs.basic_simulator.BaseTestFixture;
import com.gos.codelabs.basic_simulator.subsystems.ChassisSubsystem;
import edu.wpi.first.math.util.Units;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class AutoDriveStraightDistanceCommandTest  extends BaseTestFixture {

    @Test
    public void testExecution() {
        double goal = Units.inchesToMeters(20);
        try (ChassisSubsystem chassis = new ChassisSubsystem()) {
            AutoDriveStraightDistanceCommand command = new AutoDriveStraightDistanceCommand(chassis, goal);
            command.schedule();

            runCycles(400);

            assertFalse(command.isScheduled());
            assertEquals(goal, chassis.getAverageDistance(), AutoDriveStraightDistanceCommand.ALLOWABLE_ERROR);

        }
    }
}
