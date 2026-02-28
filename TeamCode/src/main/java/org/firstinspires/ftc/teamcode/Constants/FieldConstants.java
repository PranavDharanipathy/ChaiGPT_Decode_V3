package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Systems.CurrentAlliance;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class FieldConstants {

    public static Pose RELOCALIZATION_POSE = new Pose(0, -59.75, Math.toRadians(-90));

    public static Pose BLUE_BASE_POSE = new Pose();
    public static Pose RED_BASE_POSE = new Pose();

    /**
     * x is forward-backward with forward being positive and backward being negative
     * <p>
     * y is left-right with left being positive and right being negative
     * **/
    @SuppressWarnings("all")
    public enum GoalCoordinates {

        //        CLOSE ALLIANCE       CLOSE OPPONENT            FAR
        RED(new Pose(72,76), new Pose(72,72), new Pose(67,72)),
        BLUE(new Pose(-72,76), new Pose(-72,72), new Pose(-67,72));

        private Pose closeAlliance;
        private Pose closeOpponent;
        private Pose far;

        GoalCoordinates(Pose closeAlliance, Pose closeOpponent, Pose far) {

            this.closeAlliance = closeAlliance;
            this.closeOpponent = closeOpponent;
            this.far = far;
        }

        /// Sets the current the close and far {@link GoalCoordinate}
        public void setGoalCoordinates(Pose closeAlliance, Pose closeOpponent, Pose far) {

            this.closeAlliance = closeAlliance;
            this.closeOpponent = closeOpponent;
            this.far = far;
        }

        public Pose getCloseAllianceCoordinate() {
            return closeAlliance;
        }
        public Pose getCloseOpponentCoordinate() {
            return closeOpponent;
        }

        public Pose getCloseCoordinate(double y, GoalCoordinates allianceUsingGoalCoordinates) {

            boolean isOpponent = allianceUsingGoalCoordinates == BLUE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent ? closeOpponent : closeAlliance;
        }

        public Pose getCloseCoordinate(double y, CurrentAlliance.ALLIANCE alliance) {

            boolean isOpponent = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent ? closeOpponent : closeAlliance;
        }

        public Pose getFarCoordinate() {
            return far;
        }

        // (lateral) y value after which (once y is greater) close goal coordinate switches from alliance to opponent
        public static double RED_CLOSE_GOAL_COORDINATE_SWITCH = -25;
        public static double BLUE_CLOSE_GOAL_COORDINATE_SWITCH = 25;

        public void setRedCloseGoalCoordinateSwitch(double redCloseGoalCoordinateSwitch) {
            RED_CLOSE_GOAL_COORDINATE_SWITCH = redCloseGoalCoordinateSwitch;
        }

        public void setBlueCloseGoalCoordinateSwitch(double blueCloseGoalCoordinateSwitch) {
            BLUE_CLOSE_GOAL_COORDINATE_SWITCH = blueCloseGoalCoordinateSwitch;
        }

        public static boolean onAllianceSide(double y, CurrentAlliance.ALLIANCE alliance) {

            boolean isAlliance = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? y > RED_CLOSE_GOAL_COORDINATE_SWITCH : y < BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isAlliance;
        }

        public boolean onAllianceSide(double y) {

            boolean isAlliance = this == BLUE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isAlliance;
        }

        public static boolean onOpponentSide(double y, CurrentAlliance.ALLIANCE alliance) {

            boolean isOpponent = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent;
        }

        public boolean onOpponentSide(double y) {

            boolean isOpponent = this == BLUE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent;
        }
    }

    public enum GoalCoordinatesForDistance {

        RED(new Pose(-60, 60)),
        BLUE(new Pose(60, 60));

        private Pose coord;

        GoalCoordinatesForDistance(Pose coord) {
            this.coord = coord;
        }

        public Pose getCoordinate() {
            return coord;
        }
    }


}
