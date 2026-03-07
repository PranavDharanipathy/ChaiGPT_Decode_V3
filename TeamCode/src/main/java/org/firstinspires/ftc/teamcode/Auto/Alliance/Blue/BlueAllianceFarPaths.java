package org.firstinspires.ftc.teamcode.Auto.Alliance.Blue;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Config
public class BlueAllianceFarPaths {

    public static double INTAKE_DIST = 26;
    public static double WALL_CLEARANCE = 8.5;

    public static PathChain intakeFirst(Follower follower) {

        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                new Pose(INTAKE_DIST, WALL_CLEARANCE)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    public static PathChain returnGeneral(Follower follower) {

        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                new Pose(56, WALL_CLEARANCE)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    public static PathChain intakeGeneral(Follower follower) {

        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                new Pose(INTAKE_DIST, WALL_CLEARANCE)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }

    public static PathChain movementRP(Follower follower) {

        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                new Pose(33, 9)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }
}
