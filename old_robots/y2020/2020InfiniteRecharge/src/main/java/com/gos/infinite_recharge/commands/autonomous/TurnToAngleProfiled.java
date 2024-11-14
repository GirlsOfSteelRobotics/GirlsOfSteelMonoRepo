package com.gos.infinite_recharge.commands.autonomous;

import com.gos.infinite_recharge.Constants;
import com.gos.infinite_recharge.subsystems.Chassis;
import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

@SuppressWarnings("removal")
public class TurnToAngleProfiled extends ProfiledPIDCommand {

    private static final GosDoubleProperty AUTO_KP = new GosDoubleProperty(
            true, "TurnToAngleProfiledKp", 0.05);
    private static final GosDoubleProperty AUTO_KI = new GosDoubleProperty(
        true, "TurnToAngleProfiledKi", 0.05);
    private static final GosDoubleProperty AUTO_KD = new GosDoubleProperty(
        true, "TurnToAngleProfiledKd", 0.05);
    private static final GosDoubleProperty MAX_TURN_RATE_DEG_PER_SEC = new GosDoubleProperty(
        true, "MaxTurnRateDegPerSec", Constants.MAX_TURN_RATE_DEG_PER_S);
    private static final GosDoubleProperty MAX_TURN_ACCELERATION_DEG_PER_SEC_SQUARED = new GosDoubleProperty(
        true, "MaxTurnAccelerationDegPerSecSquared", Constants.MAX_TURN_ACCELERATION_DEG_PER_S_SQUARED);

    // private final Chassis m_chassis;
    // private final double m_angle;
    // private final double m_allowableError;

    public TurnToAngleProfiled(double targetAngleDegrees, Chassis chassis) {
        super(new ProfiledPIDController(Constants.TURN_KP, Constants.TURN_KI, Constants.TURN_KD,
                new TrapezoidProfile.Constraints(MAX_TURN_RATE_DEG_PER_SEC.getValue(),
                        MAX_TURN_ACCELERATION_DEG_PER_SEC_SQUARED.getValue())),
                // Close loop on heading
                chassis::getHeading,
                // Set reference to target
                targetAngleDegrees,
                // Pipe output to turn robot
                (output, setpoint) -> chassis.setSteer(output),
                // Require the drive
                chassis);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the
        // setpoint before it is considered as having reached the reference

        getController().setTolerance(Constants.TURN_TOLERANCE_DEG, Constants.TURN_RATE_TOLERANCE_DEG_PER_S);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();

        getController().setP(AUTO_KP.getValue());
        getController().setI(AUTO_KI.getValue());
        getController().setD(AUTO_KD.getValue());

        getController().setConstraints(new TrapezoidProfile.Constraints(MAX_TURN_RATE_DEG_PER_SEC.getValue(),
                MAX_TURN_ACCELERATION_DEG_PER_SEC_SQUARED.getValue()));

    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atGoal();
    }
}
