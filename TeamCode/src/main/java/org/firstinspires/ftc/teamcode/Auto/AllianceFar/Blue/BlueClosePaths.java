package org.firstinspires.ftc.teamcode.Auto.AllianceFar.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClosePaths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain firstCornerIntake;
    public PathChain firstCornerReturn;

    public BlueClosePaths(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.415, 8.585),

                                new Pose(63.366, 16.659)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(115))

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(63.366, 16.659),
                                new Pose(55.585, 24.585),
                                new Pose(52.439, 38.780),
                                new Pose(18.976, 35.780),
                                new Pose(9.561, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.561, 36.000),

                                new Pose(62.366, 6.951)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(115))

                .build();

        firstCornerIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(62.366, 6.951),

                                new Pose(3.756, 7.730)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        firstCornerReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3.756, 7.730),

                                new Pose(62.293, 6.902)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}
  