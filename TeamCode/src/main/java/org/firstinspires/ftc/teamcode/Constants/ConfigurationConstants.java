package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Systems.TurretBasePIDFSCoefficients;

import java.util.ArrayList;
import java.util.List;

public class ConfigurationConstants {

    public static DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction TRANSFER_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static Servo.Direction BLOCKER_SERVO_DIRECTION = Servo.Direction.FORWARD;

    /// Index 0 is the left crservo.
    /// <p>
    /// Index 1 is the right crservo.
    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    /// Index 0 is the left servo.
    /// <p>
    /// Index 1 is the right servo.
    public static Servo.Direction[] HOOD_ANGLER_SERVO_DIRECTIONS = {
            Servo.Direction.FORWARD,
            Servo.Direction.FORWARD
    };

    /// Index 0 is the left motor.
    /// <p>
    /// Index 1 is the right motor.
    public static DcMotorSimple.Direction[] FLYWHEEL_MOTOR_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD
    };

    public static double[] TRANSFER_PDF_COEFFICIENTS = {0, 0, 0};

    public static double FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT = 1534;
    public static double FLYWHEEL_SHAFT_DIAMETER = 8;
    public static double FLYWHEEL_MOTOR_CORE_VOLTAGE = 12;
    public static double FLYWHEEL_MOTOR_RPM = 6000;

    //TODO update flywheel PID
    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.0005,
            0.0044,
            0.000025,
            0.00031,
            0.00003,
            0.00042,
            0.000001,
            0.83,
            167,
            80,
            0.67,
            0.23,
            -0.25, 1,
            -0.3, 0.3,
            -0.2, 0.2
    );

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 8;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 8;

    public static TurretBasePIDFSCoefficients TURRET_PIDFS_COEFFICIENTS = new TurretBasePIDFSCoefficients(
            0.0,
            new double[] {0, 0},
            new double[] {0, 0},
            0.0,
            null,
            0.0,
            new double[] {1000, 1000},
            new double[] {0, 0},
            new double[] {0, 0},
            new double[] {1, 1},
            1,
            new double[] {1, 1},
            0,
            -0.33,
            0.33
    );

    public static List<Double> TURRET_PD_POSITIONS = new ArrayList<>(List.of(-11000.0,  -10000.0,    -9000.0,   -8000.0,   -7000.0,   -6000.0,    -5000.0,    -4000.0,    -3000.0,   -2000.0,  -1000.0,     0.0,      1000.0,    2000.0,    3000.0,   4000.0,   5000.0,   6000.0,    7000.0,    8000.0,   9000.0,   10000.0,   11000.0,    12000.0,   13000.0));
    public static List<Double> TURRET_KPS =          new ArrayList<>(List.of(0.000285,   0.000279,   0.000285,   0.0002,    0.0002,   0.00014,    0.00011,    0.00009,    0.00008,   0.00007,  0.00005,   0.00005,   0.000092,  0.0000975,  0.0001,   0.00011,  0.00012,  0.00012,  0.00012,    0.00011,  0.00011,  0.0001,    0.00014,    0.0002,    0.00033));
    public static List<Double> TURRET_KDS =          new ArrayList<>(List.of(  0.003,    0.00318,     0.003,      0.003,    0.003,     0.0028,     0.0024,     0.0022,     0.0022,    0.002,    0.002,     0.002,     0.00385,   0.00375,   0.0032,   0.00353,  0.0035,   0.0032,    0.0032,     0.003,    0.003,   0.0035,    0.0031,     0.0025,     0.002));

    public static List<Double> TURRET_FEEDFORWARD_POSITIONS = new ArrayList<>(List.of(-110000.0, -10000.0,   -9000.0,    -8000.0,    -7000.0,    -6000.0,    -5000.0,     -4000.0,    -3000.0,   -2000.0,    -1000.0,     0.0,       1000.0,    2000.0,    3000.0,     4000.0,     5000.0,   6000.0,     7000.0,     8000.0,   9000.0,    10000.0,   11000.0,    12000.0,   13000.0));
    public static List<Double> TURRET_KFS =                   new ArrayList<>(List.of( 0.000003, 0.0000021,  0.0000021,  0.000002,  0.00000175, 0.00000185,  0.0000018,  0.00000185, 0.0000025, 0.0000025,   0.000003,  0.000011,   0.000008,  0.000007,  0.0000032,   0.000003,  0.000003, 0.000003,  0.0000031,  0.0000029, 0.000003,  0.00000305, 0.0000035,  0.000004,  0.000006));

    public static double TURRET_KF_RESISTANCE_ENGAGE_ERROR = 3000;

}
