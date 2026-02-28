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

    public static Servo.Direction BLOCKER_SERVO_DIRECTION = Servo.Direction.REVERSE;

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
            Servo.Direction.REVERSE,
            Servo.Direction.FORWARD
    };

    public static double LEFT_HOOD_ALIGNMENT_OFFSET = 0.0175; //added to servo position entered into HoodAngler
    public static double RIGHT_HOOD_ALIGNMENT_OFFSET = 0; //added to servo position entered into HoodAngler


    /// Index 0 is the left motor.
    /// <p>
    /// Index 1 is the right motor.
    public static DcMotorSimple.Direction[] FLYWHEEL_MOTOR_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD
    };

    public static double[] TRANSFER_PDF_COEFFICIENTS = {0.000075, 0.00000136, 0.000428};

    public static double FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT = 1534;
    public static double FLYWHEEL_SHAFT_DIAMETER = 8;
    public static double FLYWHEEL_MOTOR_CORE_VOLTAGE = 12;
    public static double FLYWHEEL_MOTOR_RPM = 6000;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.0005,
            0.00003,
            0.0000175,
            0.000285,
            0.00003,
            0.0004,
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

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 10;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 10;

    public static TurretBasePIDFSCoefficients TURRET_PIDFS_COEFFICIENTS = new TurretBasePIDFSCoefficients(
            0.0,
            new double[] {5.25e-9, 5.15e-9},
            new double[] {3.37e-7, 3.365e-7},
            0.0,
            null,
            0.05,
            new double[] {650, 650},
            new double[] {0.12, 0.145},
            new double[] {200, 200},
            new double[] {0.9, 0.9},
            1,
            new double[] {-16, 1},
            0,
            -0.45,
            0.45
    );

    public static List<Double> TURRET_PD_POSITIONS = new ArrayList<>(List.of(-14000.0,   -13000.0,   -12000.0,   -11000.0,   -10000.0,   -9000.0,    -8000.0,    -7000.0,    -6000.0,    -5000.0,      -4000.0,    -3000.0,    -2000.0,     -1000.0,       0.0,        1000.0,    2000.0,     3000.0,    4000.0,     5000.0,     6000.0,    7000.0,     8000.0,     9000.0,   10000.0));
    public static List<Double> TURRET_KPS =          new ArrayList<>(List.of(0.000215,    0.0002,     0.0002,     0.00021,    0.00021,   0.00022,     0.0002,    0.00017,    0.00014,     0.0001,      0.000095,   0.000085,   0.0000845,  0.00008485,  0.0000844,   0.0000842,  0.000084,  0.0000833,  0.000082,   0.000082,   0.00008,   0.00008,    0.00008,    0.00008,   0.0001));
    public static List<Double> TURRET_KDS =          new ArrayList<>(List.of( 0.0025,     0.0049,     0.0049,     0.0048,     0.0048,    0.0042,      0.004,     0.0035,     0.0035,      0.0017,       0.002,      0.0045,    0.0045,      0.00534,     0.00532,     0.00535,   0.00535,    0.0053,    0.00533,    0.005355,   0.005275,  0.00528,    0.00505,     0.0045,    0.0015));

    public static List<Double> TURRET_FEEDFORWARD_POSITIONS = new ArrayList<>(List.of(-14000.0,   -13000.0,  -12000.0,      -11000.0,   -10000.0,   -9000.0,    -8000.0,    -7000.0,    -6000.0,    -5000.0,      -4000.0,   -3000.0,    -2000.0,    -1000.0,    0.0,     1000.0,      2000.0,     3000.0,     4000.0,    5000.0,    6000.0,      7000.0,     8000.0,     9000.0,   10000.0));
    public static List<Double> TURRET_KFS =                   new ArrayList<>(List.of(0.0000022,  0.0000022,  0.00000225,  0.00000225,  0.0000024, 0.00000285,  0.000003,  0.0000032,  0.0000031,   0.0000022,   0.0000022,  0.000002,   0.000001,  0.00000082,  0.0,   0.00000082,  0.00000082,  0.000001,   0.000001,  0.000001,  0.0000012,  0.00000135,  0.0000018,  0.000002,  0.000003));

    public static double TURRET_KF_RESISTANCE_ENGAGE_ERROR = 3000;

}
