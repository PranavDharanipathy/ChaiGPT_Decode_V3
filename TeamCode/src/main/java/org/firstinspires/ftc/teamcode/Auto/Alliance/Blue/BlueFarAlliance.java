package org.firstinspires.ftc.teamcode.Auto.Alliance.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.RobotNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.FlywheelNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.HoodNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TurretNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "BLUE FAR ALLIANCE", group = "FAR_AUTO", preselectTeleOp = "V3TeleOp_BLUE")
@Config
public class BlueFarAlliance extends NextFTCOpMode {
    private Telemetry telemetry;

    public static double TURRET_POSITION = -6500;

    public static double hoodPos = 0.21;
    public static double FLYWHEEL_VELOCITY = 1350;

    public BlueFarAlliance() {
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

        PedroComponent.follower().setStartingPose(new Pose(56, 8, Math.PI));

        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getCurrentVelocity());
        telemetry.addData("turret start pos: ", TurretNF.INSTANCE.turret.startPosition);


        telemetry.update();
    }

    private ElapsedTime universalTimer = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {

        telemetry.clearAll();

        //setup
        FlywheelNF.INSTANCE.flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
        IntakeNF.INSTANCE.intake.setPower(IntakeConstants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(hoodPos);
        TurretNF.INSTANCE.turret.setPosition(TURRET_POSITION);

        //auto
        universalTimer.reset();
        new SequentialGroup(
                new ParallelRaceGroup(
                        auto(),
                        new WaitUntil(() -> universalTimer.milliseconds() > 29_000)

                ),

                TurretNF.INSTANCE.goToHomePositionCmd(),
                TransferNF.INSTANCE.idleFull(),
                IntakeNF.INSTANCE.reverse(),

                new FollowPath(BlueAllianceFarPaths.movementRP(PedroComponent.follower()), true)
        ).schedule();

    }


    @Override
    public void onUpdate() {

        telemetry.addData("Blocked state", TransferNF.INSTANCE.blocker.getState());

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

        RobotNF.robot.end();
        TurretNF.INSTANCE.turret.setPosition(TurretNF.INSTANCE.turret.startPosition);
    }


    private Command auto() {

        return new SequentialGroup(

                TransferNF.INSTANCE.idle(),
                IntakeNF.INSTANCE.intake(),

                //PRELOAD SHOOTING
                new WaitUntil(() -> (
                        FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= FLYWHEEL_VELOCITY - 50)
                ),
                RobotNF.robot.shootBalls(0.9),

                //FIRST INTAKE
                followCancelable(BlueAllianceFarPaths.intakeFirst(PedroComponent.follower()), 4),
                new Delay(1),
                driveShootPara(BlueAllianceFarPaths.returnGeneral(PedroComponent.follower()), 4),

                new Delay(1),

                //GENERAL INTAKE
                followCancelable(BlueAllianceFarPaths.intakeGeneral(PedroComponent.follower()), 4000),
                new Delay(1),
                driveShootPara(BlueAllianceFarPaths.returnGeneral(PedroComponent.follower()), 4000),

                new Delay(1),

                followCancelable(BlueAllianceFarPaths.intakeGeneral(PedroComponent.follower()), 4000),
                new Delay(1),
                driveShootPara(BlueAllianceFarPaths.returnGeneral(PedroComponent.follower()), 4000),

                new Delay(1),

                followCancelable(BlueAllianceFarPaths.intakeGeneral(PedroComponent.follower()), 4000),
                new Delay(1),
                driveShootPara(BlueAllianceFarPaths.returnGeneral(PedroComponent.follower()), 4000),

                new Delay(1),

                followCancelable(BlueAllianceFarPaths.intakeGeneral(PedroComponent.follower()), 4000),
                new Delay(1),
                driveShootPara(BlueAllianceFarPaths.returnGeneral(PedroComponent.follower()), 4000),

                new Delay(2),

                //RESET
                TurretNF.INSTANCE.setPosition(TurretNF.INSTANCE.turret.startPosition),
                IntakeNF.INSTANCE.reverse()
        );
    }

    private Command driveShootPara(PathChain pathChain, double timeTilCancel) {

        return new SequentialGroup(

                followCancelable(pathChain, timeTilCancel),
                new Delay(5),
                RobotNF.robot.shootBalls(1.5)
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

                        return cancel || PedroComponent.follower().atParametricEnd();
                    }
                }
        );
    }

}

