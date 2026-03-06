package org.firstinspires.ftc.teamcode.Auto.Close15.Blue;

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
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


//THIS IS THE AUTO

@Autonomous(name = "BLUE CLOSE", group = "CLOSE_AUTO", preselectTeleOp = "V3TeleOp_BLUE")
@Config
public class BlueClose extends NextFTCOpMode {
    private Telemetry telemetry;

    public static double[] TURRET_POSITIONS = {0,500,500,0};

    //CHANGED HOOD POS FROM 0.11 to 0.19(shoots slightly higher)
    public static double hoodPos = 0.19;
    public static double flywheel_target = 900;

    private BlueClosePaths paths;


    public BlueClose() {
        addComponents(
                new SubsystemComponent(
                        RobotNF.robot,
                        FlywheelNF.INSTANCE,
                        TurretNF.INSTANCE,
                        HoodNF.INSTANCE,
                        IntakeNF.INSTANCE,
                        TransferNF.INSTANCE
                ),
                new PedroComponent(DriveConstants::createAutoFollower),
                BulkReadComponent.INSTANCE
        );
    }


    public void onInit() {


        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        PedroComponent.follower().setStartingPose(new Pose(20.29268292682926, 123.12195121951221, Math.toRadians(135)));


        paths = new BlueClosePaths(PedroComponent.follower());


        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getCurrentVelocity());
        telemetry.addData("turret start pos: ", TurretNF.INSTANCE.turret.startPosition);


        telemetry.update();
    }

    private ElapsedTime universalTimer = new ElapsedTime();

    private ElapsedTime shootTime = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {


        telemetry.clearAll();

        PedroComponent.follower().setMaxPower(2);


        //setup
        FlywheelNF.INSTANCE.flywheel.setVelocity(flywheel_target, true);
        IntakeNF.INSTANCE.intake.setPower(IntakeConstants.INTAKE_POWER);
        HoodNF.INSTANCE.setPosition(hoodPos);
        TransferNF.INSTANCE.antiNormal();
        TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]);

        universalTimer.reset();

        shootTime.reset();

        //auto
        new SequentialGroup(
                new ParallelRaceGroup(

                       auto(),
                        new WaitUntil(() -> universalTimer.milliseconds() > 35_000)

                ),

                TurretNF.INSTANCE.goToHomePositionCmd(),
                FlywheelNF.INSTANCE.setVel(0, true),
                //TransferNF.INSTANCE.antiNormal(),
                IntakeNF.INSTANCE.fullReverse()

                //new FollowPath(paths.movementRP, true)
        ).schedule();

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

        //TurretNF.INSTANCE.turret.setPosition(TurretNF.INSTANCE.turret.startPosition);

        FlywheelNF.INSTANCE.flywheel.setVelocity(0, true);

        IntakeNF.INSTANCE.intake.setPower(0);
        TransferNF.INSTANCE.antiNormal();

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

                TransferNF.INSTANCE.antiVeryStrong(),




                //PRELOAD SHOOTING
                new FollowPath(paths.preload),




                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(

                                IntakeNF.INSTANCE.intake(),
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
                                TransferNF.INSTANCE.antiVeryStrong()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                //SECOND INTAKE
                new ParallelGroup(

                        IntakeNF.INSTANCE.intake(),
                        followCancelable(paths.secondIntake, 4000) ,//new FollowPath(paths.intake),

                        changeShootVel(130)
                ),

                new Delay(0.12
                ),

                //SECOND RETURN




                new FollowPath(paths.secondReturn, true),
                new Delay(0.6),
                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(

                                IntakeNF.INSTANCE.intake(),
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
                                TransferNF.INSTANCE.antiVeryStrong()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),


                //FIRST GATE
                new FollowPath(paths.gate),

                new Delay(1.29),

                new FollowPath(paths.gateReturn),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                IntakeNF.INSTANCE.intake(),                                //TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),

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
                                TransferNF.INSTANCE.antiVeryStrong()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),


                //FIRST INTAKE



                new FollowPath(paths.firstIntake),

                        IntakeNF.INSTANCE.intake(),


                //FIRST RETURN
                //followCancelable(paths.firstReturn, 4000),//new FollowPath(paths.intake),
                new FollowPath(paths.firstReturn, true),


                //changeShootVel(40),

                new Delay(1),
                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                IntakeNF.INSTANCE.intake(),
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
                                TransferNF.INSTANCE.antiVeryStrong()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                //FIRST GATE
                new FollowPath(paths.gate),

                new Delay(3.5),

                new FollowPath(paths.gateReturn),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                IntakeNF.INSTANCE.intake(),                                //TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),

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
                                TransferNF.INSTANCE.antiVeryStrong()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),




                //END OF SEQUENTIALGROUP




                //followCancelable(paths.secondReturn, 4500),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                IntakeNF.INSTANCE.intake(),
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
                                TransferNF.INSTANCE.antiVeryStrong()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),


                //THIRD INTAKE

                new FollowPath(paths.thirdIntake),
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
                                TransferNF.INSTANCE.antiVeryStrong()
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

        ElapsedTime timer = new ElapsedTime();

        return new SequentialGroup(

                //1

                new WaitUntil(() -> (FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= flywheel_target - 50)),

                TransferNF.INSTANCE.transfer(),
                new Delay(1)
        );
    }

    public Command gating(PathChain cancelablePath, double timeTilCancel, double allowedTimeAtGateWhenFollowing, double uncancelledHoldTime, double cancelledHoldTime) {

        return new Command() {

            final String[] STAGES = {"START", "FOLLOW", "TIME_STOP", "HOLD_TIME_DECISION", "DELAY", "DONE"};

            String stage = STAGES[0];

            private ElapsedTime openGateTimer = new ElapsedTime();

            private double timeAlreadyAtGate = 0;

            private double holdTime = 0;

            @Override
            public boolean isDone() {

                switch (stage) {

                    case "START":

                        openGateTimer.reset();
                        PedroComponent.follower().followPath(cancelablePath);
                        stage = STAGES[1];
                        break;

                    case "FOLLOW":

                        boolean cancel = false;

                        if (openGateTimer.milliseconds() >= timeTilCancel) {

                            cancel = true;
                            PedroComponent.follower().breakFollowing();
                        }

                        if (cancel || PedroComponent.follower().atParametricEnd()) {
                            stage = STAGES[2];
                        }
                        break;

                    case "TIME_STOP":

                        timeAlreadyAtGate = openGateTimer.milliseconds();
                        stage = STAGES[3];
                        break;

                    case "HOLD_TIME_DECISION":

                        if (timeAlreadyAtGate > allowedTimeAtGateWhenFollowing) {
                            holdTime = cancelledHoldTime;
                        }
                        else {
                            holdTime = uncancelledHoldTime;
                        }

                        openGateTimer.reset();

                        stage = STAGES[4];
                        break;

                    case "DELAY":

                        if (openGateTimer.milliseconds() > holdTime) stage = STAGES[5];
                        break;
                }

                return stage.equals("DONE");
            }
        };

    }

}


