package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Systems.TurretBasePIDFSCoefficients;

public class ConfigurationConstants {

    public static DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction TRANSFER_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static Servo.Direction BLOCKER_SERVO_DIRECTION = Servo.Direction.FORWARD;

    /// Index 0 is the left crservo.
    /// <p>
    /// Index 1 is the right crservo.
    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE
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
            -0.1, 1,
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

}
