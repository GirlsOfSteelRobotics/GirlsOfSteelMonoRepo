package org.usfirst.frc.team3504.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.usfirst.frc.team3504.robot.Robot;
import org.usfirst.frc.team3504.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Flap extends Subsystem {

    private final CANTalon flapTalon;
    private static final double maxEncoder = 360; //max encoder val
    private double encOffsetValue = 0;

    public Flap(){
        flapTalon = new CANTalon(RobotMap.FLAP_MOTOR);
        addChild("Talon", flapTalon);

        if(RobotMap.USING_LIMIT_SWITCHES) {
            flapTalon.configFwdLimitSwitchNormallyOpen(false);
            flapTalon.configRevLimitSwitchNormallyOpen(false);
        }
        else {
            flapTalon.enableLimitSwitch(false, false);
        }
        flapTalon.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setTalon(double speed){
        flapTalon.set(speed);
    }

    public void stopTalon(){
        flapTalon.set(0.0);
    }

    public double getThrottle() {
        return Robot.oi.getOperatorStickThrottle();
    }

    public double getMaxEnc() {
        return maxEncoder;
    }

    //assuming that going forward will raise the flap and going backwards will lower the flap
    public boolean getTopLimitSwitch(){
        return flapTalon.isFwdLimitSwitchClosed() == 0;
    }
    public boolean getBottomLimitSwitch(){
        return flapTalon.isRevLimitSwitchClosed() == 0;
    }

    public double getFlapEncoder() {
        return flapTalon.getEncPosition();
    }

    public double getFlapEncoderDistance() {
        return (getFlapEncoder() - encOffsetValue); //TODO: know how far encoder is
    }

    public void resetDistance() {
        encOffsetValue = getFlapEncoder();
    }
}
