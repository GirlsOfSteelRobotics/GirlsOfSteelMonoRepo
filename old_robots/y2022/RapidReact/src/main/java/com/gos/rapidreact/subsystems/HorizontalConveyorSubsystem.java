package com.gos.rapidreact.subsystems;


import com.gos.rapidreact.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HorizontalConveyorSubsystem extends SubsystemBase {

    public static final double HORIZONTAL_CONVEYOR_TELEOP_MOTOR_SPEED = 0.5;
    public static final double HORIZONTAL_CONVEYOR_AUTO_MOTOR_SPEED = 1;


    private final SparkMax m_leader;


    public HorizontalConveyorSubsystem() {
        m_leader = new SparkMax(Constants.HORIZONTAL_CONVEYOR_LEADER_SPARK, MotorType.kBrushless);
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getHorizontalConveyorSpeed() {
        return m_leader.getAppliedOutput();
    }

    public void forwardHorizontalConveyorMotor() {
        if (DriverStation.isTeleop()) {
            m_leader.set(HORIZONTAL_CONVEYOR_TELEOP_MOTOR_SPEED);
        }
        if (DriverStation.isAutonomous()) {
            m_leader.set(HORIZONTAL_CONVEYOR_AUTO_MOTOR_SPEED);
        }
    }

    public void backwardHorizontalConveyorMotor() {
        if (DriverStation.isTeleop()) {
            m_leader.set(-HORIZONTAL_CONVEYOR_TELEOP_MOTOR_SPEED);
        }
        if (DriverStation.isAutonomous()) {
            m_leader.set(-HORIZONTAL_CONVEYOR_AUTO_MOTOR_SPEED);
        }
    }

    public void stopHorizontalConveyorMotor() {
        m_leader.set(0);
    }

}
