package org.firstinspires.ftc.teamcode.Systems;

import com.chaigptrobotics.shenanigans.Peak;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocityTracker;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.EffectivelySubsystem;

import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY;

import androidx.annotation.NonNull;

import java.util.function.DoubleUnaryOperator;

@Peak
public class Shooter implements EffectivelySubsystem {

    private BetterGamepad controller1, controller2;

    public Flywheel flywheel;

    public TurretBase turret;

    public HoodAngler hoodAngler;

    private Follower follower;
    private Camera camera;
    private PoseVelocityTracker poseVelocityTracker;

    public void provideComponents(Flywheel flywheel, TurretBase turret, HoodAngler hoodAngler, Follower follower, Camera unstartedCamera, BetterGamepad controller1, BetterGamepad controller2) {

        this.follower = follower;
        camera = unstartedCamera;
        poseVelocityTracker = new PoseVelocityTracker(follower);

        this.flywheel = flywheel;

        this.turret = turret;

        this.hoodAngler = hoodAngler;

        this.controller1 = controller1;
        this.controller2 = controller2;

    }

    public enum ZONE {
        CLOSE("CLOSE"), FAR("FAR");

        private String string;

        ZONE(String string) {
            this.string = string;
        }

        @NonNull
        public String toString() {
            return string;
        }
    }

    private ZONE flywheelTargetVelocityZone = ZONE.FAR;

    private double turretStartPosition;

    private FieldConstants.GoalCoordinates goalCoordinates;

    /// Primarily for modification purposes, however can totally be used for telemetry, haptics, etc.
    public FieldConstants.GoalCoordinates accessGoalCoordinates() {
        return goalCoordinates;
    }

    public void switchAlliance(CurrentAlliance.ALLIANCE alliance) {

        if (alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE) {

            camera.pipelineSwitch(CameraConstants.PIPELINES.BLUE_PIPELINE.getPipelineIndex());
            goalCoordinates = FieldConstants.GoalCoordinates.BLUE;
        }
        else {

            camera.pipelineSwitch(CameraConstants.PIPELINES.RED_PIPELINE.getPipelineIndex());
            goalCoordinates = FieldConstants.GoalCoordinates.RED;
        }

    }

    public void start(CurrentAlliance.ALLIANCE alliance) {

        camera.start();

        goalCoordinates = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? FieldConstants.GoalCoordinates.BLUE : FieldConstants.GoalCoordinates.RED;

        CameraConstants.PIPELINES pipeline = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? CameraConstants.PIPELINES.BLUE_PIPELINE : CameraConstants.PIPELINES.RED_PIPELINE;
        camera.pipelineSwitch(pipeline.getPipelineIndex());

        turretPosition = turretStartPosition = turret.startPosition;

        EOAPose = follower.getPose(); //starting pose
        relocalization(EOAPose); //runs full multi-sensor localization

        flywheel.reset();
    }

    private boolean shooterToggle = false;

    private double hoodPosition;

    private double turretPosition;
    private double turretAcceleration = 0;

    private double robotYawRad;
    public double tt;

    private Pose goalCoordinate;

    /// For hysteresis control on the turret, this is the robot's position on the field at a point in time in the future.
    public Pose futureRobotPose;
    public Pose currentRobotPose;
    public Pose turretPose;
    public Pose EOAPose;
    private double turretTimeLookahead = 0;
    private boolean isTurretLookingAhead = false; //initially the bot is stationary

    private double distanceToGoal;

    public void update() {

        //getting robot pose
        if (controller2.main_buttonHasJustBeenPressed) relocalization(FieldConstants.RELOCALIZATION_POSE);

        follower.update();
        poseVelocityTracker.update();
        TurretHelper.update(turret);

        robotYawRad = follower.getPose().getHeading(); //rev9AxisImuWrapped.getYaw(AngleUnit.RADIANS);
        PoseVelocity robotVelocity = poseVelocityTracker.getPoseVelocity();
        double translationalVelocity = Calculations.getRobotTranslationalVelocity(robotVelocity.getXVelocity(), robotVelocity.getYVelocity());
        //turret
        turretAcceleration = TurretHelper.getAcceleration(AngleUnit.RADIANS);
        double turretCurrentPosition = turret.getCurrentPosition(); //used to calculate turret pose

        if (controller2.dpad_leftHasJustBeenPressed) {
            turretStartPosition+=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }
        else if (controller2.dpad_rightHasJustBeenPressed) {
            turretStartPosition-=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }

        currentRobotPose = follower.getPose();

        //hysteresis control is only used if the robot is moving fast enough
        isTurretLookingAhead = Math.abs(translationalVelocity) > TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY[0] || Math.abs(robotVelocity.getAngularVelocity()) > TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY[1];

        if (isTurretLookingAhead) {

            if (!THCTuning) {
                turretTimeLookahead = 0;//1.3;
            }
            else { //are tuning
                turretTimeLookahead = customTHC;
            }

            futureRobotPose = Calculations.getFutureRobotPose(
                    turretTimeLookahead,
                    currentRobotPose,
                    robotVelocity
            );
        }
        else {

            turretTimeLookahead = 0;
            futureRobotPose = currentRobotPose;
        }

        turretPose = Calculations.getTurretPoseFromBotPose(futureRobotPose, turretCurrentPosition, turretStartPosition);

        //changing the coordinate that the turret aims at based on targeted zones determined by distance
        if (currentRobotPose.getX() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            goalCoordinate = goalCoordinates.getCloseCoordinate(futureRobotPose.getY(), goalCoordinates);
        }
        else {
            goalCoordinate = goalCoordinates.getFarCoordinate();
        }

        double angleToGoal;

        angleToGoal = Calculations.getAngleToGoal(turretPose.getX(), turretPose.getY(), goalCoordinate);

        double rawtt = angleToGoal - Math.toDegrees(robotYawRad);
        tt = route(rawtt);

        turretPosition = tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;
        double targetPosition = turretPosition;

        if (controller2.left_trigger(GeneralConstants.TRIGGER_THRESHOLD)) turret.setPosition(turretStartPosition);
        else turret.setPosition(targetPosition);

        //flywheel
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        // setting flywheel velocity
        if (controller1.yHasJustBeenPressed) { //close

            flywheelTargetVelocityZone = ZONE.CLOSE;
            controller2.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }
        else if (controller1.bHasJustBeenPressed) { //far

            flywheelTargetVelocityZone = ZONE.FAR;
            controller2.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }

        if (shooterToggle) flywheel.setVelocity(getFlywheelTargetVelocity(), true);
        else flywheel.setVelocity(0, true);

        if (controller2.right_trigger(GeneralConstants.TRIGGER_THRESHOLD)) {
            if (flywheel.getMotorEnabled()) flywheel.runMotor(Flywheel.RunningMotor.DISABLE);
            flywheel.setPower(1);
        }
        else if (!flywheel.getMotorEnabled()) {
            flywheel.runMotor(Flywheel.RunningMotor.ENABLE);
        }

        // if turret is within an acceptable amount of error, the controller is rumbled.
        if (turret.getPositionError() < ShooterConstants.TURRET_TARGET_POSITION_ACCEPTABLE_ERROR_MARGIN) {
            controller1.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }
        else {
            controller1.stopRumble();
        }

        //hood

        if (flywheelTargetVelocityZone == ZONE.FAR) hoodPosition = ShooterConstants.HOOD_FAR_POSITION;
        else {

            FieldConstants.GoalCoordinatesForDistance goalCoordinatesForDistance =
                    goalCoordinates == FieldConstants.GoalCoordinates.BLUE
                            ? FieldConstants.GoalCoordinatesForDistance.BLUE
                            : FieldConstants.GoalCoordinatesForDistance.RED;

            distanceToGoal = Calculations.getDistanceFromGoal(turretPose.getX(), turretPose.getY(), goalCoordinatesForDistance.getCoordinate());

            hoodPosition = ShooterConstants.HOOD_CLOSE_POSITION; //Models.getCloseHoodPositionFromRegression(distanceToGoal);
        }

        //updating
        hoodAngler.setSafePosition(hoodPosition);
        turret.update();
        flywheel.update();

    }

    private double getFlywheelTargetVelocity() {

        double flywheelTargetVelocity;

        if (flywheelTargetVelocityZone == ZONE.FAR) {
            flywheelTargetVelocity = ShooterConstants.FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        else if (goalCoordinates.onOpponentSide(futureRobotPose.getY())) {
            flywheelTargetVelocity = ShooterConstants.OPPONENT_SIDE_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        else if (distanceToGoal < ShooterConstants.CLOSE_SIDE_SWITCH) {
            flywheelTargetVelocity = ShooterConstants.CLOSER_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        else {
            flywheelTargetVelocity = ShooterConstants.FARTHER_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY;
        }
        return flywheelTargetVelocity;
    }

    private double route(double rawtt) {

        if (rawtt >= ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && rawtt <= ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) return rawtt; //no need to reroute

        double[] reroutes = {rawtt - 360, rawtt + 360};
        if (reroutes[0] >= ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && reroutes[0] <= ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[0];
        }
        else if (reroutes[1] >= ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES && reroutes[1] <= ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) {
            return reroutes[1];
        }

        // go to the closest limit if target position is outside the min and max
        else if (rawtt < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES) return ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES;
        else return ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
    }

    private void relocalization(Pose reZeroPose) {
        follower.setPose(reZeroPose);
    }

    public ZONE getZone() {
        return flywheelTargetVelocityZone;
    }

    public boolean isTurretLookingAhead() {
        return isTurretLookingAhead;
    }

    public double getTHCLookahead() {
        return turretTimeLookahead;
    }

    private boolean THCTuning = false;

    public void setTHCTuning(boolean tuning) {
        THCTuning = tuning;
    }

    private double customTHC;

    public void provideCustomTHCLookahead(DoubleUnaryOperator THCoperation) {
        customTHC = THCoperation.applyAsDouble(turretAcceleration);
    }

}
