package com.gos.reefscape.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.rev.properties.pid.RevPidPropertyBuilder;
import com.gos.lib.rev.swerve.config.RevSwerveModuleConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.swerve.SwerveModuleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SwerveModuleSimWrapper;

public class RevSwerveModule {
    private static final double GEAR_REDUCTION_DRIVE = 5.36;
    private static final double GEAR_REDUCTION_STEER = 18.75;

    private final String m_name;

    private final CANcoder m_absoluteEncoder;
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_steerEncoder;
    private final SparkFlex m_motorDrive;
    private final SparkFlex m_motorSteer;
    private final SparkClosedLoopController m_drivePID;
    private final SparkClosedLoopController m_steerPID;

    private SwerveModuleSimWrapper m_simWrapper;

    private final PidProperty m_drivePidProperties;
    private final PidProperty m_steerPidProperties;



    public RevSwerveModule(String name, int absoluteEncoderID, int motorDriveID, int motorSteerID) {
        m_name = name;

        m_absoluteEncoder = new CANcoder(absoluteEncoderID);
        m_motorDrive = new SparkFlex(motorDriveID, MotorType.kBrushless);
        m_motorSteer = new SparkFlex(motorSteerID, MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(60);
        driveConfig.inverted(false);

        driveConfig.encoder.positionConversionFactor(1 / GEAR_REDUCTION_DRIVE);
        driveConfig.encoder.velocityConversionFactor(1 / GEAR_REDUCTION_DRIVE / 60);




        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.idleMode(IdleMode.kBrake);
        steerConfig.smartCurrentLimit(60);
        steerConfig.inverted(false);

        steerConfig.encoder.positionConversionFactor(1 / GEAR_REDUCTION_STEER);
        steerConfig.encoder.velocityConversionFactor(1 / GEAR_REDUCTION_STEER / 60);

        m_motorDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorSteer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_drivePidProperties = new RevPidPropertyBuilder("SwerveDrivePid", false, m_motorDrive, driveConfig, ClosedLoopSlot.kSlot0)
            .addP(0)
            .addFF(0)
            .build();

        m_steerPidProperties = new RevPidPropertyBuilder("SwerveSteerPid", false, m_motorSteer, steerConfig, ClosedLoopSlot.kSlot0)
            .addP(0)
            .addFF(0)
            .build();



        m_driveEncoder = m_motorDrive.getEncoder();
        m_steerEncoder = m_motorSteer.getEncoder();

        m_drivePID = m_motorDrive.getClosedLoopController();
        m_steerPID = m_motorSteer.getClosedLoopController();


        if (RobotBase.isSimulation()) {
            DCMotor turningMotor = DCMotor.getNEO(1);
            DCMotor drivingMotor = DCMotor.getNEO(1);
            SwerveModuleSim moduleSim = new SwerveModuleSim(
                turningMotor,
                drivingMotor,
                RevSwerveModuleConstants.WHEEL_DIAMETER_METERS / 2,
                RevSwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR,
                GEAR_REDUCTION_DRIVE,
                1.0,
                1.8, // Seems fishy
                1.1,
                0.8,
                16.0,
                0.001
            );
            m_simWrapper = new SwerveModuleSimWrapper(
                moduleSim,
                new RevMotorControllerSimWrapper(m_motorDrive, drivingMotor),
                new RevMotorControllerSimWrapper(m_motorSteer, turningMotor),
                RevEncoderSimWrapper.create(m_motorDrive),
                RevEncoderSimWrapper.create(m_motorSteer),
                RevSwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI,
                false);
        }
    }

    public void drive(SwerveModuleState state) {
        state.optimize(Rotation2d.fromDegrees(getSteerAngle()));
        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        m_steerPID.setReference(state.angle.getDegrees(), ControlType.kPosition);

        SmartDashboard.putNumber(m_name + "Goal Velocity", state.speedMetersPerSecond);
        SmartDashboard.putNumber(m_name + "Current Velocity", m_driveEncoder.getVelocity());

        SmartDashboard.putNumber(m_name + "Goal Position", state.angle.getDegrees());
        SmartDashboard.putNumber(m_name + "Current Position", m_steerEncoder.getPosition());
    }

    public double getAbsoluteEncoderPosition() {
        return m_absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_driveEncoder.getVelocity();

    }

    public double getSteerAngle() {
        return m_steerEncoder.getPosition();
    }

    public SwerveModuleSimWrapper getSimWrapper() {
        return m_simWrapper;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            Rotation2d.fromDegrees(m_steerEncoder.getPosition()));
    }

    public void periodic() {
        m_steerPidProperties.updateIfChanged();
        m_drivePidProperties.updateIfChanged();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getSteerAngle()));
    }
}
