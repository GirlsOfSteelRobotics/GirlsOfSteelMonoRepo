package com.gos.lib.rev.swerve;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;

@SuppressWarnings("PMD.DataClass")
public final class RevSwerveModuleConstants {
    // Modified version of https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

    public final DriveMotor m_driveMotorType;
    public final double m_drivingMotorReduction;
    public final double m_driveWheelFreeSpeedRps;

    public final double m_drivingEncoderPositionFactor;
    public final double m_drivingEncoderVelocityFactor;

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static final class VortexMotorConstants {
        public static final double FREE_SPEED_RPM = 6784;
    }

    public enum DriveMotorPinionTeeth {
        T12(12),
        T13(13),
        T14(14),
        T15(15),
        T16(16);

        public final int m_teeth;

        DriveMotorPinionTeeth(int teeth) {
            m_teeth = teeth;
        }
    }

    public enum DriveMotorSpurTeeth {
        T19(19),
        T20(20),
        T21(21),
        T22(22);

        public final int m_teeth;

        DriveMotorSpurTeeth(int teeth) {
            m_teeth = teeth;
        }
    }

    public enum DriveMotor {
        NEO(NeoMotorConstants.FREE_SPEED_RPM / 60),
        VORTEX(VortexMotorConstants.FREE_SPEED_RPM / 60);

        public final double m_freeSpeedRps;

        DriveMotor(double freeSpeedRps) {
            m_freeSpeedRps = freeSpeedRps;
        }
    }

    public RevSwerveModuleConstants(DriveMotor driveMotor, DriveMotorPinionTeeth pinionTeeth, DriveMotorSpurTeeth spurTeeth) {
        m_driveMotorType = driveMotor;

        m_drivingMotorReduction = (45.0 * spurTeeth.m_teeth) / (pinionTeeth.m_teeth * 15);
        m_driveWheelFreeSpeedRps = (driveMotor.m_freeSpeedRps * WHEEL_CIRCUMFERENCE_METERS)
            / m_drivingMotorReduction;

        m_drivingEncoderPositionFactor = (WHEEL_DIAMETER_METERS * Math.PI)
            / m_drivingMotorReduction; // meters
        m_drivingEncoderVelocityFactor = ((WHEEL_DIAMETER_METERS * Math.PI)
            / m_drivingMotorReduction) / 60.0; // meters per second
    }
}
