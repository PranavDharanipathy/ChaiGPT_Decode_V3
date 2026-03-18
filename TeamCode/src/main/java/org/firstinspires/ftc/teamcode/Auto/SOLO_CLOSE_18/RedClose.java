package org.firstinspires.ftc.teamcode.Auto.SOLO_CLOSE_18;

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
import org.firstinspires.ftc.teamcode.Auto.Subsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.TurretNF;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

// THIS IS THE AUTO
@Autonomous(name = "(SOLO) RED CLOSE 18", group = "SOLO", preselectTeleOp = "V3TeleOp_RED")
@Config
public class RedClose extends NextFTCOpMode {
    private Telemetry telemetry;

    public static double[] TURRET_POSITIONS = {5900,3800,3700,4800};

    //CHANGED HOOD POS FROM 0.11 to 0.19(shoots slightly higher)
    public static double hoodPos = 0.19;
    public double flywheel_target = 940;

    private RedCloseMirroredPaths paths;


    public RedClose() {
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

        flywheel_target = 940;


        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        PedroComponent.follower().setStartingPose(new Pose(123.70731707317074, 123.12195121951221, Math.toRadians(45)));


        paths = new RedCloseMirroredPaths(PedroComponent.follower());


        telemetry.addData("flywheel vel: ", FlywheelNF.INSTANCE.flywheel.getCurrentVelocity());
        telemetry.addData("turret start pos: ", TurretNF.INSTANCE.turret.startPosition);

TransferNF.INSTANCE.block();
        telemetry.update();
    }

    private ElapsedTime universalTimer = new ElapsedTime();

    private ElapsedTime shootTime = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {


        telemetry.clearAll();


        //setup
        //FlywheelNF.INSTANCE.setVel(flywheel_target,true);
        FlywheelNF.INSTANCE.flywheel.setVelocity(flywheel_target, true);
        IntakeNF.INSTANCE.intake.setPower(IntakeConstants.INTAKE_POWER);
        HoodNF.INSTANCE.setPosition(hoodPos);
        TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]);
        TransferNF.INSTANCE.block();

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
                TransferNF.INSTANCE.block(),
                IntakeNF.INSTANCE.reverse()

                //new FollowPath(paths.movementRP, true)
        ).schedule();

    }


    @Override
    public void onUpdate() {

        FlywheelNF.INSTANCE.flywheel.setVelocity(flywheel_target, true);


        telemetry.addData("turret start", TurretNF.INSTANCE.turret.startPosition);


        telemetry.addData("current path: ", PedroComponent.follower().getCurrentPath());

        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("current pose: " ,PedroComponent.follower().getPose());

        telemetry.addData("path completion: ", PedroComponent.follower().getPathCompletion());

        telemetry.addLine();



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
        TransferNF.INSTANCE.block();

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

                TransferNF.INSTANCE.block(),

                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),
                //PRELOAD SHOOTING
                new FollowPath(paths.preload, true),


                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[0]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.block()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                //FIRST INTAKE

                changeShootVel(20),

                new FollowPath(paths.secondIntake),

                changeShootVel(-40),

                new Delay(0.25),
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),


                //FIRST RETURN
                //followCancelable(paths.firstReturn, 4000),//new FollowPath(paths.intake),
                new FollowPath(paths.secondReturn, true),


                changeShootVel(80),

                new Delay(1),
                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(
                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[1]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.block()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),

                changeShootVel(-10),
                //THIRD INTAKE

                followCancelable(paths.gate,5000),

                new Delay(0.3),
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),

                //new TurnTo(turnTo),


                new Delay(1.3),
                followCancelable(paths.gateReturn, 5000),
                new Delay(0.6),

                changeShootVel(20),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.block()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)
                        //END OF SEQUENTIALGROUP
                ),


                new FollowPath(paths.gate, true),


                changeShootVel(-20),

                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[2]),
                changeShootVel(30),

                new Delay(1.8),
                followCancelable(paths.gateReturn, 4000),
                new Delay(0.5),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(

                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.block()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),


                new FollowPath(paths.firstIntake),
                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),


                new Delay(0.52),
                followCancelable(paths.firstReturn, 5000),
                new Delay(1),


                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(

                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.block()
                        ),
                        new WaitUntil(() -> shootTime.seconds() > 9)


                        //END OF SEQUENTIALGROUP
                ),


                new FollowPath(paths.thirdIntake),

                new Delay(0.92),
                followCancelable(paths.thirdReturn, 5000),
                new Delay(2.5),

                resetShootTimer(),
                new ParallelRaceGroup(

                        new SequentialGroup(

                                TurretNF.INSTANCE.setPosition(TURRET_POSITIONS[3]),

                                shootBalls(
                                        new double[] {0.35, 0.375, 0.4},
                                        new double[] {0, 0},
                                        new double[] {0.4, 0.4},
                                        300
                                ),
                                TransferNF.INSTANCE.block()
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

                //new InstantCommand(() -> PedroComponent.follower().followPath(pathChain)),
                new FollowPath(pathChain, true),
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

                new WaitUntil(() -> (FlywheelNF.INSTANCE.flywheel.getCurrentVelocity() >= flywheel_target - 100)),

                TransferNF.INSTANCE.transfer(),
                new Delay(1.4)
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


