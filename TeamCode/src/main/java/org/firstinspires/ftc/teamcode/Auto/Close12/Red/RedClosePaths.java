package org.firstinspires.ftc.teamcode.Auto.Close12.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class RedClosePaths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain thirdIntake;
    public PathChain thirdReturn;

    public RedClosePaths(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.707, 123.122),
                                new Pose(113.134, 112.268),
                                new Pose(105.537, 103.756)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(105.537, 103.756),
                                new Pose(117.566, 100.071),
                                new Pose(120.922, 94.899),
                                new Pose(119.902, 89.878),
                                new Pose(123.390, 84.488)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.390, 84.488),

                                new Pose(85.195, 89.488)
                        )
                ).setLinearHeadingInterpolation(0, Math.toRadians(45))

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(81.195, 75.488),
                                new Pose(83.317, 68.878),
                                new Pose(109.573, 69.451),
                                new Pose(95.671, 58.305),
                                new Pose(123.061, 64.573),
                                new Pose(134.634, 60.439)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.634, 60.439),

                                new Pose(95.512, 92.829)
                        )
                ).setLinearHeadingInterpolation(0, Math.toRadians(40))

                .build();

        thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(95.512, 92.829),
                                new Pose(72.476, 40.854),
                                new Pose(107.573, 26.927),
                                new Pose(110.012, 39.000),
                                new Pose(133.171, 36.341)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(133.171, 36.341),

                                new Pose(93.854, 95.683)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(45))
                                )
                        )
                )
                .build();
    }
}
  