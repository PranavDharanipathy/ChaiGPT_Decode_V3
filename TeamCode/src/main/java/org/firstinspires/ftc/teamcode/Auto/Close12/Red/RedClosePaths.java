package org.firstinspires.ftc.teamcode.Auto.Close12.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedClosePaths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain line6;
    public PathChain thirdReturn;

    public RedClosePaths(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.707, 123.122),
                                new Pose(114.695, 111.683),
                                new Pose(107.683, 108.244)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(107.683, 108.244),
                                new Pose(117.566, 100.071),
                                new Pose(120.922, 94.899),
                                new Pose(119.902, 89.878),
                                new Pose(120.390, 84.488)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.390, 84.488),

                                new Pose(96.707, 100.415)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(96.707, 100.415),
                                new Pose(83.317, 68.878),
                                new Pose(113.280, 59.890),
                                new Pose(95.671, 58.305),
                                new Pose(108.817, 60.671),
                                new Pose(137.171, 59.902)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(137.171, 59.902),

                                new Pose(91.512, 94.829)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        line6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(91.512, 94.829),
                                new Pose(72.476, 40.854),
                                new Pose(107.573, 26.927),
                                new Pose(110.012, 39.000),
                                new Pose(133.171, 36.341)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.171, 36.341),

                                new Pose(93.854, 95.683)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
  