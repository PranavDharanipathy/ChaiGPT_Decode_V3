package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import com.chaigptrobotics.shenanigans.No_u;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;


public class TransferNF implements Subsystem {

    //Doesn't allow objects to be created
    private TransferNF() {}

    public static final TransferNF INSTANCE = new TransferNF();

    public DcMotorEx transfer;

    @Override
    public void initialize() {

        transfer = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Command transfer() {
        return new InstantCommand(() -> transfer.setVelocity(2300));
    }

    public Command antiStrong() {
        return new InstantCommand(() -> transfer.setVelocity(-450));
    }

    public Command antiVeryStrong() {
        return new InstantCommand(() -> transfer.setVelocity(-2000));
    }

    public Command antiNormal() {
        return new InstantCommand(() -> transfer.setVelocity(-100));
    }

    public Command idle() {
        return new InstantCommand(() -> transfer.setVelocity(0));
    }

    public void end() {
        transfer.setVelocity(0);
    }

    @Override
    public void periodic() {

    }
}
