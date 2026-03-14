package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Auto.Subsystems.FlywheelNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.HoodNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TurretNF;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

/// He da big R, big R for Robot
public class RobotNF extends SubsystemGroup {


    private RobotNF() {

        super(
                IntakeNF.INSTANCE,
                TransferNF.INSTANCE,
                HoodNF.INSTANCE,
                TurretNF.INSTANCE,
                FlywheelNF.INSTANCE
        );
    }

    public static final RobotNF robot = new RobotNF();

    //transfer
    public final Command shootBalls(double transferTime) {

        return new SequentialGroup(

                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.idle()
        );
    }

    public final Command shootBallsAtParametricEnd(double transferTime, PathChain pathChain) {

        return new SequentialGroup(

                new WaitUntil(() -> pathChain.lastPath().isAtParametricEnd()),

                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.idle()
        );
    }

    public final Command shootBalls(double transferTime, double distance, PathChain pathChain) {

        return new SequentialGroup(

                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() <= distance),

                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.idleFull()
        );
    }

    //hood
    public final Command hoodTo(double position) {
        return new InstantCommand(() -> HoodNF.INSTANCE.setPosition(position));
    }

    /// Intakes first and then outtakes
    public final Command intakeClearingSpecial(double outtakeTime) {

        return new SequentialGroup(

                IntakeNF.INSTANCE.reverse(),
                new Delay(outtakeTime),
                IntakeNF.INSTANCE.intake()
        );
    }

    /// No turret
    public final void end() {
        IntakeNF.INSTANCE.end();
        TransferNF.INSTANCE.end();
        FlywheelNF.INSTANCE.end();
    }
}
