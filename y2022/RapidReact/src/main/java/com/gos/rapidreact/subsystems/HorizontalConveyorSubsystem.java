package com.gos.rapidreact.subsystems;


import com.gos.rapidreact.Constants;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HorizontalConveyorSubsystem extends SubsystemBase {

    public static final int HORIZONTAL_CONVEYOR_MOTOR_SPEED = 1;

    private final SimableCANSparkMax m_leader;
    private final SimableCANSparkMax m_follower;


    public HorizontalConveyorSubsystem() {
        m_leader = new SimableCANSparkMax(Constants.HORIZONTAL_CONVEYOR_LEADER_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_follower = new SimableCANSparkMax(Constants.HORIZONTAL_CONVEYOR_FOLLOWER_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);

        m_follower.follow(m_leader, false);
    }

    public void forwardHorizontalConveyorMotor() {
        m_leader.set(HORIZONTAL_CONVEYOR_MOTOR_SPEED);
    }

    public void backwardHorizontalConveyorMotor() {
        m_leader.set(-HORIZONTAL_CONVEYOR_MOTOR_SPEED);
    }

    public void stopHorizontalConveyorMotor() {
        m_leader.set(0);
    }

}

