package org.firstinspires.ftc.teamcode.Auto.Close12.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
                                new Pose(30.866, 112.268),
                                new Pose(38.854, 104.537)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.805, 96.537),
                                new Pose(26.434, 100.071),
                                new Pose(23.078, 94.899),
                                new Pose(24.098, 89.878),
                                new Pose(21.610, 82.488)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.610, 84.488),

                                new Pose(58.805, 89.488)
                        )
                ).setLinearHeadingInterpolation(Math.PI, Math.toRadians(135))

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.805, 89.488),
                                new Pose(60.683, 69.878),
                                new Pose(30.720, 61.890),
                                new Pose(48.329, 60.305),
                                new Pose(35.183, 60.671),
                                new Pose(19.000, 65.707)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 59.707),

                                new Pose(52.488, 94.829)
                        )

                ).setReversed()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.tangent

                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(135))
                                )
                        )
                )

                .build();

        thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.488, 94.829),
                                new Pose(71.524, 40.854),
                                new Pose(36.427, 26.927),
                                new Pose(33.988, 39.000),
                                new Pose(18.829, 36.341)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.829, 36.341),

                                new Pose(50.146, 95.683)
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
                                        HeadingInterpolator.constant(Math.toRadians(135))
                                )
                        )
                )

                .build();
    }
}
  