package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Calculations {

    public static Pose convertPose3DtoPedroPose(Pose3D pose3d) {

        double x = MathUtil.metersToInches(pose3d.getPosition().x);
        double y = MathUtil.metersToInches(pose3d.getPosition().y);
        double yaw = pose3d.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(x, y, yaw).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public static Pose getTurretPoseFromBotPose(Pose botPose, double turretPositionTicks, double turretStartPositionTicks) {

        double reZeroedTurretTicks = turretPositionTicks - turretStartPositionTicks;
        double turretRotation = Math.toRadians(reZeroedTurretTicks / ShooterConstants.TURRET_TICKS_PER_DEGREE);

        double turretHeading = botPose.getHeading() + turretRotation + Math.toRadians(ShooterConstants.TURRET_ANGULAR_OFFSET);

        double turretX = botPose.getX() + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.cos(botPose.getHeading()));
        double turretY = botPose.getY() + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.sin(botPose.getHeading()));

        return new Pose(
                turretX,
                turretY,
                turretHeading
        );
    }

    public static Pose getFutureRobotPose(double t, Pose currentRobotPose, PoseVelocity poseVelocity) {

        return new Pose(
                currentRobotPose.getX() + (t * poseVelocity.getXVelocity()),
                currentRobotPose.getY() + (t * poseVelocity.getYVelocity()),
                currentRobotPose.getHeading() + (t * poseVelocity.getAngularVelocity())
        );
    }

    public static double getRobotTranslationalVelocity(double xVelocity, double yVelocity) {
        return Math.hypot(xVelocity, yVelocity);
    }

    /// For turret hysteresis control - the amount of time in the future where the robot's pose
    /// will be predicted based on its current pose and velocity as well as the turret's acceleration.
    /// @param turretAcceleration is in rad/sec^2
    public static double getTurretFuturePosePredictionTime(double turretAcceleration) {

        final double NORMAL_ACCEL = Math.toRadians(7.2);
        final double NORMAL = 1.5;
        final double MAX = 2.5;

        final double turretAccel = Math.abs(turretAcceleration);

        double futurePredictionTime = NORMAL * (1 + Math.sqrt(NORMAL_ACCEL / (turretAccel + 1e-3)));

        return MathUtil.clamp(futurePredictionTime, NORMAL, MAX);
    }

}
