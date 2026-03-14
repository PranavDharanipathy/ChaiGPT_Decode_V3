package org.firstinspires.ftc.teamcode.Auto.Solo.Blue;

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
import org.firstinspires.ftc.teamcode.data.EOALocalization;

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
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "(SOLO) BLUE CLOSE 12", group = "CLOSE_AUTO", preselectTeleOp = "V3TeleOp_BLUE")
@Config
public class BlueSoloClose extends NextFTCOpMode {
    private Telemetry telemetry;

    public static double[] TURRET_POSITIONS = {0,0,0,0, 300};


    //CHANGED HOOD POS FROM 0.11 to 0.19(shoots slightly higher)
    public static double hoodPos = 0.19;
    public double flywheel_target = 900;

    private BlueSoloClosePaths paths;


    public BlueSoloClose() {
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

        PedroComponent.follower().setStartingPose(new Pose(20.29268292682926, 123.12195121951221, Math.toRadians(135)));


        paths = new BlueSoloClosePaths(PedroComponent.follower());


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
        FlywheelNF.INSTANCE.flywheel.setVelocity(flywheel_target, true);
        IntakeNF.INSTANCE.intake.setPower(IntakeConstants.INTAKE_POWER);
        HoodNF.INSTANCE.hood.setPosition(hoodPos);
        TurretNF.INSTANCE.turret.setPosition(TURRET_POSITIONS[0]);

        universalTimer.reset();

        shootTime.reset();

        //auto
        auto().schedule();

    }


    @Override
    public void onUpdate() {

        FlywheelNF.INSTANCE.flywheel.setVelocity(flywheel_target, true);


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

        FlywheelNF.INSTANCE.flywheel.setVelocity(0, true);

        IntakeNF.INSTANCE.intake.setPower(0);
        TransferNF.INSTANCE.idleFull();

    }

    Command resetShootTimer() {
        return new InstantCommand(
                () -> shootTime.reset());
    }

    Command changeShootVel(double increment) {
        return new InstantCommand(
                () -> flywheel_target += increment
        );
    }

    private Command auto() {


        return new SequentialGroup(

                TransferNF.INSTANCE.idle(),




                //PRELOAD SHOOTING
                new FollowPath(paths.preload, true),


                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                //TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= flywheel_target - 1000)

                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),


                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),


                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.idleFull()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),



                //FIRST INTAKE

                changeShootVel(20),



                new FollowPath(paths.secondIntake),

                new Delay(0.7),


                //FIRST RETURN
                //followCancelable(paths.firstReturn, 4000),//new FollowPath(paths.intake),
                new FollowPath(paths.secondReturn, true),


                changeShootVel(80),

                new Delay(1),
                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= flywheel_target - 1000)
                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),


                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),
                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.idle()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                //SECOND INTAKE
                new ParallelGroup(
                        RobotNF.robot.intakeClearingSpecial(0.25),
                        followCancelable(paths.firstIntake, 4000) //new FollowPath(paths.intake),
                ),

                new Delay(0.6),

                //SECOND RETURN

                new FollowPath(paths.firstReturn, true),
                new Delay(0.4),

                //followCancelable(paths.secondReturn, 4500),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                // TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= flywheel_target - 1000)
                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.idle()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                changeShootVel(-10),
                //THIRD INTAKE

                new FollowPath(paths.thirdIntake),

                new Delay(0.92),
                followCancelable(paths.thirdReturn, 5000),
                new Delay(1),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                //TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),

                                new WaitUntil(() -> (
                                        FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= flywheel_target - 1000)
                                        //&& Math.abs(TurretNF.INSTANCE.turret.getError()) < 200
                                ),
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.idle()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),



                //SET TURRET TO END POS
                TurretNF.INSTANCE.setPosition(TurretNF.INSTANCE.turret.startPosition),
                IntakeNF.INSTANCE.reverse()

        );
    }


    // compensate paths fo rstart pos
    //makes sure it shoots  3 balls





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

    private Command shootBalls(double[] transferTime, double[] minTimeBetweenTransfers, double[] maxTimeBetweenTransfers, double flywheelVelMargin) {

        return new SequentialGroup(

                new WaitUntil(() -> (FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= flywheel_target - 100)),

                TransferNF.INSTANCE.transfer(),
                new Delay(1)
        );
    }


}