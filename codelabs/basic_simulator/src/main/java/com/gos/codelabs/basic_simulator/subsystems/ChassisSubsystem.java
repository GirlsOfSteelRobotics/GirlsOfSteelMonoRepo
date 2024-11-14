package com.gos.codelabs.basic_simulator.subsystems;

import com.gos.codelabs.basic_simulator.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.module_wrappers.wpi.ADXRS450GyroWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

public class ChassisSubsystem extends SubsystemBase implements AutoCloseable {

    private final SparkMax m_leftDriveA;
    private final SparkMax m_leftDriveB;
    private final SparkMax m_rightDriveA;
    private final SparkMax m_rightDriveB;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private final DifferentialDrive m_differentialDrive;
    private final DifferentialDriveOdometry m_odometry;
    private final Field2d m_field;

    private final ADXRS450_Gyro m_gyro;

    private DifferentialDrivetrainSimWrapper m_simulator;

    public static final class DrivetrainConstants {

        public static final DCMotor DRIVE_GEARBOX = DCMotor.getCIM(2);
        public static final double K_DRIVE_GEARING = 8;

        public static final double K_TRACK_WIDTH_METERS = 0.69;
        public static final double K_WHEEL_DIAMETER_METERS = 0.15;

        public static final double KS_VOLTS = 0.22;
        public static final double KV_VOLT_SECONDS_PER_METER = 1.98;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.2;
        public static final double KV_VOLT_SECONDS_PER_RADIAN = 2.5;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_RADIAN = 0.8;

        public static final LinearSystem<N2, N2, N2> K_DRIVETRAIN_PLANT =
                LinearSystemId.identifyDrivetrainSystem(KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER,
                        KV_VOLT_SECONDS_PER_RADIAN, KA_VOLT_SECONDS_SQUARED_PER_RADIAN);

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);

        public static DifferentialDrivetrainSim createSim() {
            return new DifferentialDrivetrainSim(
                    K_DRIVETRAIN_PLANT,
                    DRIVE_GEARBOX,
                    K_DRIVE_GEARING,
                    K_TRACK_WIDTH_METERS,
                    K_WHEEL_DIAMETER_METERS / 2.0,
                    Constants.SIMULATE_SENSOR_NOISE ? VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005) : null); // NOPMD
        }

        private DrivetrainConstants() {

        }
    }

    public ChassisSubsystem() {

        m_leftDriveA = new SparkMax(Constants.CAN_CHASSIS_LEFT_A, MotorType.kBrushless);
        m_leftDriveB = new SparkMax(Constants.CAN_CHASSIS_LEFT_B, MotorType.kBrushless);
        SparkMaxConfig leftDriveAConfig = new SparkMaxConfig();
        SparkMaxConfig leftDriveBConfig = new SparkMaxConfig();
        leftDriveBConfig.follow(m_leftDriveA);

        m_rightDriveA = new SparkMax(Constants.CAN_CHASSIS_RIGHT_A, MotorType.kBrushless);
        m_rightDriveB = new SparkMax(Constants.CAN_CHASSIS_RIGHT_B, MotorType.kBrushless);
        SparkMaxConfig rightDriveAConfig = new SparkMaxConfig();
        SparkMaxConfig rightDriveBConfig = new SparkMaxConfig();
        rightDriveBConfig.follow(m_rightDriveA);
        m_rightDriveA.setInverted(true);

        m_leftEncoder = m_leftDriveA.getEncoder();
        m_rightEncoder = m_rightDriveA.getEncoder();

        m_gyro = new ADXRS450_Gyro();

        m_differentialDrive = new DifferentialDrive(m_leftDriveA, m_rightDriveA);

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
        m_field = new Field2d();
        SmartDashboard.putData(m_field);

        if (RobotBase.isSimulation()) {
            m_simulator = new DifferentialDrivetrainSimWrapper(
                    DrivetrainConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_leftDriveA, DrivetrainConstants.DRIVE_GEARBOX),
                    new RevMotorControllerSimWrapper(m_rightDriveA, DrivetrainConstants.DRIVE_GEARBOX),
                    RevEncoderSimWrapper.create(m_leftDriveA),
                    RevEncoderSimWrapper.create(m_rightDriveA),
                    new ADXRS450GyroWrapper(m_gyro));
            m_simulator.setRightInverted(false);

            m_differentialDrive.setSafetyEnabled(false);
        }

        m_leftDriveA.configure(leftDriveAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftDriveB.configure(leftDriveBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightDriveA.configure(rightDriveAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightDriveA.configure(rightDriveAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void close() {
        m_leftDriveA.close();
        m_leftDriveB.close();
        m_rightDriveA.close();
        m_rightDriveB.close();
        m_differentialDrive.close();
        m_gyro.close();
    }

    public void arcadeDrive(double speed, double steer) {
        // TODO implement
    }

    public void stop() {
        // TODO implement
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        m_simulator.update();
    }

    public double getHeading() {
        // TODO implement
        return 0;
    }

    public double getLeftDistance() {
        // TODO implement
        return 0;
    }

    public double getRightDistance() {
        // TODO implement
        return 0;
    }

    public double getAverageDistance() {
        // TODO implement
        return 0;
    }
}
