package com.gos.chargedup.subsystems;


import com.gos.lib.properties.PidProperty;
import com.gos.lib.rev.RevPidPropertyBuilder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.swerve.SwerveModuleSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SwerveModuleSimWrapper;


public class SwerveDriveModules {

    // TODO these are SDS mk4i ratios
    private static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    private static final double TURNING_GEAR_RATIO = (50.0 / 14.0) * (60.0 / 10.0);
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
    private static final double DRIVE_ENCODER_CONSTANT = (1.0 / DRIVE_GEAR_RATIO) * WHEEL_DIAMETER_METERS * Math.PI;

    private final SimableCANSparkMax m_wheel;
    private final SimableCANSparkMax m_azimuth;

    public final RelativeEncoder m_wheelEncoder;

    public final RelativeEncoder m_azimuthRelativeEncoder;
    public final AbsoluteEncoder m_azimuthAbsoluteEncoder;

    private final PidProperty m_wheelPID;

    private final String m_moduleName;

    private final PidProperty m_azimuthPID;

    private final SparkMaxPIDController m_wheelPidController;

    private final SparkMaxPIDController m_azimuthPidController;

    private SwerveModuleSimWrapper m_simWrapper;

    public SwerveDriveModules(int wheelId, int azimuthId, String moduleName, double zeroOffset) {
        m_wheel = new SimableCANSparkMax(wheelId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_azimuth = new SimableCANSparkMax(azimuthId, CANSparkMaxLowLevel.MotorType.kBrushless);

        m_wheel.restoreFactoryDefaults();
        m_azimuth.restoreFactoryDefaults();

        m_wheel.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_azimuth.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_wheel.setSmartCurrentLimit(50);
        m_azimuth.setSmartCurrentLimit(20);

        m_wheelPidController = m_wheel.getPIDController();
        m_azimuthPidController = m_azimuth.getPIDController();

        m_azimuthRelativeEncoder = m_azimuth.getEncoder();
        m_azimuthRelativeEncoder.setPositionConversionFactor(360 / TURNING_GEAR_RATIO);
        m_azimuthRelativeEncoder.setVelocityConversionFactor(360 / TURNING_GEAR_RATIO / 60);

        m_azimuthAbsoluteEncoder = m_azimuth.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_azimuthAbsoluteEncoder.setPositionConversionFactor(360);
        m_azimuthAbsoluteEncoder.setVelocityConversionFactor(360 / 60.0);

        m_wheelEncoder = m_wheel.getEncoder();
        m_wheelEncoder.setPositionConversionFactor(DRIVE_ENCODER_CONSTANT);
        m_wheelEncoder.setVelocityConversionFactor(DRIVE_ENCODER_CONSTANT / 60);


        m_azimuthAbsoluteEncoder.setZeroOffset(zeroOffset);

        m_azimuthPidController.setFeedbackDevice(m_azimuthAbsoluteEncoder);
        m_azimuthPidController.setPositionPIDWrappingEnabled(true);
        m_azimuthPidController.setPositionPIDWrappingMinInput(0);
        m_azimuthPidController.setPositionPIDWrappingMinInput(360);

        m_wheelPID = new RevPidPropertyBuilder("Wheel PID", false, m_wheelPidController, 0)
            .addP(0)
            .addD(0)
            .addFF(0)
            .build();

        m_azimuthPID = new RevPidPropertyBuilder("Azimuth PID", false, m_azimuthPidController, 0)
            .addP(0)
            .addD(0)
            .build();

        if (RobotBase.isSimulation()) {
            SwerveModuleSim moduleSim = new SwerveModuleSim(
                DCMotor.getNEO(1),
                DCMotor.getNEO(1),
                WHEEL_DIAMETER_METERS / 2,
                TURNING_GEAR_RATIO,
                DRIVE_GEAR_RATIO
            );
            m_simWrapper = new SwerveModuleSimWrapper(
                moduleSim,
                new RevMotorControllerSimWrapper(m_wheel),
                new RevMotorControllerSimWrapper(m_azimuth),
                RevEncoderSimWrapper.create(m_wheel),
                RevEncoderSimWrapper.create(m_azimuth));
        }

        m_moduleName = moduleName;

        m_wheel.burnFlash();
        m_azimuth.burnFlash();
    }

    private Rotation2d getAzimuthEncoderRotation2D() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromDegrees(m_azimuthAbsoluteEncoder.getPosition());
        }
        else {
            return Rotation2d.fromDegrees(m_azimuthRelativeEncoder.getPosition());
        }
    }

    public SwerveModuleSimWrapper getSimWrapper() {
        return m_simWrapper;
    }

    public void setState(SwerveModuleState state) {
        m_azimuthPID.updateIfChanged();
        m_wheelPID.updateIfChanged();
        m_wheelPidController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_azimuthPidController.setReference(state.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        
        System.out.println("SPD" + state.speedMetersPerSecond);

        SmartDashboard.putNumber(m_moduleName + " Goal Angle: ", state.angle.getDegrees());
        // SmartDashboard.putNumber(m_moduleName + " Current Angle: ", m_azimuth.getEncoder().getPosition());
        SmartDashboard.putNumber(m_moduleName + " Goal Velocity: ", state.speedMetersPerSecond);
        SmartDashboard.putNumber(m_moduleName + " Current Velocity: ", m_wheel.getEncoder().getVelocity());

        SmartDashboard.putNumber(m_moduleName + " Abs Encoder Pos", m_azimuthAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber(m_moduleName + " Rel Encoder Pos", m_azimuthRelativeEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_wheelEncoder.getVelocity(), getAzimuthEncoderRotation2D());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_wheelEncoder.getPosition(), getAzimuthEncoderRotation2D());
    }


}

