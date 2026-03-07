package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.TRANSFER_VELOCITY;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Blocker;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
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
        transfer.setDirection(ConfigurationConstants.TRANSFER_MOTOR_DIRECTION);
        blocker = new Blocker(ActiveOpMode.hardwareMap().get(Servo.class, MapSetterConstants.blockerServoDeviceName));
    }

    public Command unblock() {
        return new InstantCommand(() -> blocker.setState(Blocker.BlockerState.CLEAR));
    }

    public Command block() {
        return new InstantCommand(() -> blocker.setState(Blocker.BlockerState.BLOCK));
    }


    public Command actualTransfer() {
        return new InstantCommand(() ->transfer.setVelocity(TRANSFER_VELOCITY));
    }




    public Command transfer() {
        return new SequentialGroup(

                unblock(),
                actualTransfer()
        );

    }
    public Command idle() {
        return new SequentialGroup(
                actualTransfer(),
                block()
        );
    }

    public Command anti() {
        return new SequentialGroup(
                actualTransfer(),
                block()
        );
    }
    public Command idleFull() {
        return new InstantCommand(() -> {
            blocker.setState(Blocker.BlockerState.BLOCK);
            transfer.setVelocity(0);
        });
    }

    public void end() {
        blocker.setState(Blocker.BlockerState.BLOCK);
        transfer.setVelocity(0);
    }

    @Override
    public void periodic() {



    }
}
