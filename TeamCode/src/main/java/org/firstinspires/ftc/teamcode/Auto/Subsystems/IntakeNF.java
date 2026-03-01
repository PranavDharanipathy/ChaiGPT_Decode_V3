package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


public class IntakeNF implements Subsystem {

    //Doesn't allow objects to be created
    private IntakeNF() {}

    public static final IntakeNF INSTANCE = new IntakeNF();

    public MotorEx intake;

    //CODE NOT ADDED
   // private LiftPTO liftPTO; //intentionally private

    @Override
    public void initialize() {

        intake = new MotorEx("intake");
        intake.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);


        //CODE NOT ADDEED
        //liftPTO = new LiftPTO(ActiveOpMode.hardwareMap());
        //liftPTO.setState(LiftPTO.PTOState.DISENGAGE);
    }

    public Command customPower(double power) {
        return new SetPower(intake, power);
    }

    public Command intake() {
        return new SetPower(intake, IntakeConstants.INTAKE_POWER);
    }

    public Command reverse() {
        return new SetPower(intake, IntakeConstants.REVERSE_INTAKE_POWER);
    }

    public Command fullReverse() {
        return new SetPower(intake, -1);
    }

    public void end() {
        intake.setPower(0);
    }

    @Override
    public void periodic() {

    }
}
