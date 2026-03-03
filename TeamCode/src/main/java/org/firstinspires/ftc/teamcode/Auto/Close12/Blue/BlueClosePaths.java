package org.firstinspires.ftc.teamcode.Auto.Close12.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClosePaths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain thirdIntake;
    public PathChain thirdReturn;

    public BlueClosePaths(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.293, 123.122),
                                new Pose(29.865, 112.804),
                                new Pose(36.317, 108.244)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(36.317, 108.244),
                                new Pose(24.585, 93.854),
                                new Pose(27.951, 105.317),
                                new Pose(24.098, 89.878),
                                new Pose(23.610, 84.488)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.610, 84.488),

                                new Pose(47.293, 100.415)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(47.293, 100.415),
                                new Pose(60.683, 68.878),
                                new Pose(30.720, 59.890),
                                new Pose(48.329, 58.305),
                                new Pose(35.183, 60.671),
                                new Pose(10.571, 58.295),
                                new Pose(8.697, 62.517)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.697, 62.517),

                                new Pose(52.488, 94.829)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.488, 94.829),
                                new Pose(71.524, 40.854),
                                new Pose(36.427, 26.927),
                                new Pose(33.988, 39.000),
                                new Pose(10.829, 36.341)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.829, 36.341),
                                new Pose(33.019, 70.950),
                                new Pose(42.819, 106.196)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}
