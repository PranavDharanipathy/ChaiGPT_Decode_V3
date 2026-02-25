package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Systems.FlywheelPIDVSCoefficients;
import org.firstinspires.ftc.teamcode.Systems.TurretBasePIDFSCoefficients;
import org.firstinspires.ftc.teamcode.data.EOAOffset;

import java.util.HashMap;
import java.util.Map;

public class ConfigurationConstants {

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

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients();

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
