package com.gos.rapidreact.subsystems;


import com.gos.rapidreact.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VerticalConveyorSubsystem extends SubsystemBase {

    public static final double VERTICAL_CONVEYOR_TELEOP_MOTOR_SPEED = 0.5;
    public static final double VERTICAL_CONVEYOR_AUTO_MOTOR_SPEED = 1;

    public static final double FEEDER_MOTOR_SPEED = 0.5;

    private final SparkMax m_conveyor; //multiple sets of wheels to move the cargo up
    private final SparkMax m_feeder; //needs to move for the cargo to shoot

    private final DigitalInput m_indexSensorUpper;
    private final DigitalInput m_indexSensorLower;

    // Logging
    private final NetworkTableEntry m_lowerIndexSensorEntry;
    private final NetworkTableEntry m_upperIndexSensorEntry;

    public VerticalConveyorSubsystem() {
        m_conveyor = new SparkMax(Constants.VERTICAL_CONVEYOR_SPARK, MotorType.kBrushless);
        SparkMaxConfig conveyorConfig = new SparkMaxConfig();
        conveyorConfig.idleMode(IdleMode.kBrake);
        m_conveyor.setInverted(true);

        m_feeder = new SparkMax(Constants.VERTICAL_CONVEYOR_FEEDER_SPARK, MotorType.kBrushless);
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.idleMode(IdleMode.kCoast);

        m_indexSensorUpper = new DigitalInput(Constants.INDEX_SENSOR_UPPER_VERTICAL_CONVEYOR);
        m_indexSensorLower = new DigitalInput(Constants.INDEX_SENSOR_LOWER_VERTICAL_CONVEYOR);

        m_conveyor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("VerticalConveyor");
        m_lowerIndexSensorEntry = loggingTable.getEntry("Lower Cargo Sensor");
        m_upperIndexSensorEntry = loggingTable.getEntry("Upper Cargo Sensor");
    }

    @Override
    public void periodic() {
        m_lowerIndexSensorEntry.setBoolean(getLowerIndexSensor());
        m_upperIndexSensorEntry.setBoolean(getUpperIndexSensor());

    }

    public void forwardVerticalConveyorMotor() {
        m_conveyor.set(0.5);
    }

    public void autoShootVerticalConveyorForward() {
        m_conveyor.set(VERTICAL_CONVEYOR_AUTO_MOTOR_SPEED);
    }

    public void backwardVerticalConveyorMotor() {
        if (DriverStation.isTeleop()) {
            m_conveyor.set(-VERTICAL_CONVEYOR_TELEOP_MOTOR_SPEED);
        }
        if (DriverStation.isAutonomous()) {
            m_conveyor.set(-VERTICAL_CONVEYOR_AUTO_MOTOR_SPEED);
        }
    }

    public double getVerticalConveyorSpeed() {
        return m_conveyor.getAppliedOutput();
    }

    public void stopVerticalConveyorMotor() {
        m_conveyor.set(0);
    }

    public void forwardFeedMotor() {
        m_feeder.set(FEEDER_MOTOR_SPEED);
    }

    public void backwardFeedMotor() {
        m_feeder.set(-FEEDER_MOTOR_SPEED);
    }

    public void stopFeedMotor() {
        m_feeder.set(0);
    }

    public boolean getUpperIndexSensor() {
        return !m_indexSensorUpper.get();
    }

    public boolean getLowerIndexSensor() {
        return !m_indexSensorLower.get();
    }

}
