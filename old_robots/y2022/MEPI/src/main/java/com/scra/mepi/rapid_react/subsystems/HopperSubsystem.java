package com.scra.mepi.rapid_react.subsystems;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.scra.mepi.rapid_react.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax m_hopperMotor;

    /**
     * Creates a new HopperSubsystem.
     */
    public HopperSubsystem() {
        m_hopperMotor = new SparkMax(Constants.HOPPER_SPARK, MotorType.kBrushless);
        SparkMaxConfig hopperMotorConfig = new SparkMaxConfig();
        hopperMotorConfig.idleMode(IdleMode.kCoast);
        hopperMotorConfig.smartCurrentLimit(30);
        m_hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setHopperSpeed(double speed) {
        m_hopperMotor.set(speed);
    }
}
