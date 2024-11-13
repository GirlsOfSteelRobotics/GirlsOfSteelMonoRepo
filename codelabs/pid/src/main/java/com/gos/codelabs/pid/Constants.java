// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gos.codelabs.pid;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /////////////////////////////////
    // Sensor / Actuator ports
    /////////////////////////////////

    // PWM

    // Digital IO
    public static final int DIO_LIFT_LOWER_LIMIT = 0;
    public static final int DIO_LIFT_UPPER_LIMIT = 1;

    // Solenoids
    public static final int SOLENOID_PUNCH = 0;

    // CAN
    public static final int CAN_CHASSIS_LEFT_A = 1;
    public static final int CAN_CHASSIS_LEFT_B = 2;
    public static final int CAN_CHASSIS_RIGHT_A = 3;
    public static final int CAN_CHASSIS_RIGHT_B = 4;
    public static final int CAN_LIFT_MOTOR = 5;
    public static final int CAN_SPINNING_MOTOR = 6;

    public static final boolean SIMULATE_SENSOR_NOISE = false;

    private Constants() {

    }


    public static final class DrivetrainConstants {

        public static final DCMotor DRIVE_GEARBOX = DCMotor.getCIM(3);
        public static final double DRIVE_GEARING = 8;

        public static final double TRACK_WIDTH_METERS = 0.69;
        public static final double WHEEL_DIAMETER_METERS = 0.15;

        public static final double KS_VOLTS = .15;
        public static final double KV_VOLT_SECONDS_PER_METER = .5;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.2;
        public static final double KV_VOLT_SECONDS_PER_RADIAN = 2.5;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_RADIAN = 0.8;

        public static final LinearSystem<N2, N2, N2> DRIVETRAIN_PLANT =
                LinearSystemId.identifyDrivetrainSystem(KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER,
                    KV_VOLT_SECONDS_PER_RADIAN, KA_VOLT_SECONDS_SQUARED_PER_RADIAN);

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

        public static DifferentialDrivetrainSim createSim() {
            return new DifferentialDrivetrainSim(
                DRIVETRAIN_PLANT,
                DRIVE_GEARBOX,
                DRIVE_GEARING,
                TRACK_WIDTH_METERS,
                    WHEEL_DIAMETER_METERS / 2.0,
                    SIMULATE_SENSOR_NOISE ? VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005) : null); // NOPMD
        }

        private DrivetrainConstants() {

        }
    }

    public static final class ElevatorSimConstants {
        public static final double ELEVATOR_GEARING = 10.0;
        public static final double CARRIAGE_MASS = 4.0; // kg
        public static final double MIN_ELEVATOR_HEIGHT = Units.inchesToMeters(.1);
        public static final double MAX_ELEVATOR_HEIGHT = Units.inchesToMeters(80);
        public static final DCMotor ELEVATOR_GEARBOX = DCMotor.getVex775Pro(4);

        public static final double DRUM_RADIUS = Units.inchesToMeters(2.0);

        public static ElevatorSim createSim() {

            LinearSystem<N2, N1, N2> plant = LinearSystemId.createElevatorSystem(
                ElevatorSimConstants.ELEVATOR_GEARBOX,
                ElevatorSimConstants.CARRIAGE_MASS,
                ElevatorSimConstants.DRUM_RADIUS,
                ElevatorSimConstants.ELEVATOR_GEARING
            );
            return new ElevatorSim(
                plant,
                ElevatorSimConstants.ELEVATOR_GEARBOX,
                ElevatorSimConstants.MIN_ELEVATOR_HEIGHT,
                ElevatorSimConstants.MAX_ELEVATOR_HEIGHT,
                true,
                0,
                SIMULATE_SENSOR_NOISE ?  VecBuilder.fill(0.01).getData() : new double[]{}); // NOPMD
        }

        private ElevatorSimConstants() {

        }
    }

    public static final class FlywheelSimConstants {
        public static final DCMotor GEARBOX = DCMotor.getVex775Pro(2);
        public static final double GEARING = 4;
        public static final double INERTIA = 0.03;

        public static FlywheelSim createSim() {
            LinearSystem<N1, N1, N1> plant =
                LinearSystemId.createFlywheelSystem(FlywheelSimConstants.GEARBOX, FlywheelSimConstants.INERTIA, FlywheelSimConstants.GEARING);
            return new FlywheelSim(plant, FlywheelSimConstants.GEARBOX,
                    SIMULATE_SENSOR_NOISE ? VecBuilder.fill(0.5).getData() : new double[]{}); // NOPMD
        }

        private FlywheelSimConstants() {

        }
    }
}
