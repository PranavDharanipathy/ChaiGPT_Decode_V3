package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.TRANSFER_VELOCITY;

import com.chaigptrobotics.shenanigans.No_u;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;


public class TransferNF implements Subsystem {

    //Doesn't allow objects to be created
    private TransferNF() {}

    public static final TransferNF INSTANCE = new TransferNF();

    public DcMotorEx transfer;

    public Blocker blocker;

    @Override
    public void initialize() {

        transfer = ActiveOpMode.hardwareMap().get(DcMotorEx.class, MapSetterConstants.transferMotorDeviceName);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        blocker = new Blocker(ActiveOpMode.hardwareMap().get(Servo.class, MapSetterConstants.blockerServoDeviceName));
    }

    public Command unblock() {
        return new InstantCommand(() -> blocker.setState(Blocker.BlockerState.CLEAR));
    }

    public Command block() {
        return new InstantCommand(() -> blocker.setState(Blocker.BlockerState.BLOCK));
    }


    Command actualTransfer() {
        return new InstantCommand(() ->transfer.setVelocity(TRANSFER_VELOCITY));
    }




    public Command transfer() {
        return new SequentialGroup(

                unblock(),
                actualTransfer()
        );

    }

    public Command antiStrong() {
        return new SequentialGroup(
                actualTransfer(),
                block()

        );
    }

    public Command antiVeryStrong() {
        return new SequentialGroup(
                actualTransfer(),
                block()
        );
    }

    public Command antiNormal() {
        return new SequentialGroup(
                actualTransfer(),
                block()
        );
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
