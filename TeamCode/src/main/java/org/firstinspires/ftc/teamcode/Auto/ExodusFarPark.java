package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;


@Autonomous
public class ExodusFarPark extends OpMode {

    private DcMotor lf, rf, lb, rb;

    @Override
    public void init() {

        lf = hardwareMap.get(DcMotor.class, MapSetterConstants.leftFrontMotorDeviceName);
        rf = hardwareMap.get(DcMotor.class, MapSetterConstants.rightFrontMotorDeviceName);
        lb = hardwareMap.get(DcMotor.class, MapSetterConstants.leftBackMotorDeviceName);
        rb = hardwareMap.get(DcMotor.class, MapSetterConstants.rightBackMotorDeviceName);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {

        lf.setPower(1);
        rf.setPower(1);
        lb.setPower(1);
        rb.setPower(1);
    }
}