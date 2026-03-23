package org.firstinspires.ftc.teamcode.Auto.Alliance.RedFar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.Alliance.RED.RedAllianceClosePaths;
import org.firstinspires.ftc.teamcode.Auto.RobotNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.FlywheelNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.HoodNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TurretNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.data.EOALocalization;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RED FAR", group = "FAR_AUTO", preselectTeleOp = "V3TeleOp_RED")
@Config
public class RedFar extends NextFTCOpMode {
    private Telemetry telemetry;

    public static double[] TURRET_POSITIONS = {-6000, -6230};

    public static double hoodPos = 0.19;
    public static double[] FLYWHEEL_VELOCITIES = {1750, 1450};
    public static double[] GATE_TIMES = {1.6, 1.6, 1.6};

    private RedFarPath paths;


    public RedFar() {
        addComponents(
                new SubsystemComponent(
                        RobotNF.robot,
                        FlywheelNF.INSTANCE,
                        TurretNF.INSTANCE,
                        HoodNF.INSTANCE,
                        IntakeNF.INSTANCE,
                        TransferNF.INSTANCE
                ),
                new PedroComponent(DriveConstants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }


    public void onInit() {


        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        PedroComponent.follower().setStartingPose(new Pose(55.80487804878049, 6.8292682926829364, Math.toRadians(180)));


        paths = new RedFarPath(PedroComponent.follower());


        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getCurrentVelocity());
        telemetry.addData("turret start pos: ", TurretNF.INSTANCE.turret.startPosition);


        telemetry.update();
    }

    private ElapsedTime universalTimer = new ElapsedTime();

    private ElapsedTime shootTime = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {

        telemetry.clearAll();

        //setup
        FlywheelNF.INSTANCE.flywheel.setVelocity(FLYWHEEL_VELOCITIES[0], true);
        IntakeNF.INSTANCE.intake.setPower(IntakeConstants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(hoodPos);
        TurretNF.INSTANCE.turret.setPosition(TURRET_POSITIONS[0]);

        universalTimer.reset();

        shootTime.reset();

        //auto
        auto().schedule();

    }
    Angle endTurn = Angle.fromDeg(180+45);

    @Override
    public void onUpdate() {

        telemetry.addData("turret start", TurretNF.INSTANCE.turret.startPosition);
        telemetry.addData("current path: ", PedroComponent.follower().getCurrentPath());
        telemetry.addData("flywheel target vel: ", FlywheelNF.INSTANCE.flywheel.getTargetVelocity());
        telemetry.addData("flywheel current vel: ", FlywheelNF.INSTANCE.flywheel.getCurrentVelocity());
        telemetry.addLine();

        telemetry.addData("flywheel power", FlywheelNF.INSTANCE.flywheel.getPower());
        telemetry.addLine();

        telemetry.addData("turret Current: ", TurretNF.INSTANCE.turret.getCurrentPosition());
        telemetry.addData("turret error: ", TurretNF.INSTANCE.turret.getError());
        telemetry.addData("turret target pos: ", TurretNF.INSTANCE.turret.getTargetPosition());


        telemetry.update();


    }


    @Override
    public void onStop() {

        EOALocalization.write(new Pose(), TurretNF.INSTANCE.turret.startPosition);

        RobotNF.robot.end();
        TurretNF.INSTANCE.turret.setPosition(TurretNF.INSTANCE.turret.startPosition);
    }

    private Command resetShootTimer() {
        return new InstantCommand(
                () -> shootTime.reset());
    }

    private Command auto() {

        return new SequentialGroup(

                TransferNF.INSTANCE.idle(),
                IntakeNF.INSTANCE.intake(),


                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),
                FlywheelNF.INSTANCE.setVel(FLYWHEEL_VELOCITIES[0]),

                new SequentialGroup(

                        IntakeNF.INSTANCE.intake(),

                        new WaitUntil(() -> (
                                FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= FLYWHEEL_VELOCITIES[0] - 50)
                        ),

                        RobotNF.robot.shootBalls(1)
                ),

               new FollowPath(paths.firstIntake),

                FlywheelNF.INSTANCE.setVel(FLYWHEEL_VELOCITIES[1]),
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),

                new Delay(1.2),
                new FollowPath(paths.firstReturn),




                new SequentialGroup(

                        IntakeNF.INSTANCE.intake(),

                        new WaitUntil(() -> (
                                FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= FLYWHEEL_VELOCITIES[1] - 50)
                        ),

                        RobotNF.robot.shootBalls(1)
                ),

                new FollowPath(paths.hpIntake),
                new Delay(0.8),
                new FollowPath(paths.hpReturn),

                new SequentialGroup(

                        IntakeNF.INSTANCE.intake(),

                        new WaitUntil(() -> (
                                FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= FLYWHEEL_VELOCITIES[1] - 50)
                        ),

                        RobotNF.robot.shootBalls(1)
                ),





                //RESET
                TurretNF.INSTANCE.setPosition(TurretNF.INSTANCE.turret.startPosition),
                IntakeNF.INSTANCE.reverse()
        );
    }

    private Command driveShootPara(PathChain pathChain) {

        return new ParallelGroup(

                new FollowPath(pathChain, true),

                new SequentialGroup(
                        new Delay(0.01),
                        RobotNF.robot.shootBallsAtParametricEnd(0.9, pathChain)
                )
        );
    }

    private Command followCancelable(PathChain pathChain, double timeTilCancel) {

        return new SequentialGroup(

                new InstantCommand(() -> PedroComponent.follower().followPath(pathChain)),
                new Command() {

                    private boolean firstTick = true;
                    private double startTime;

                    private boolean cancel = false;

                    @Override
                    public boolean isDone() {

                        if (firstTick) {

                            startTime = System.currentTimeMillis();
                            firstTick = false;
                        }

                        if (System.currentTimeMillis() >= timeTilCancel + startTime) {
                            cancel = true;
                            PedroComponent.follower().breakFollowing();

                        }

                        return PedroComponent.follower().atParametricEnd() || cancel;
                    }
                }
        );
    }

}
