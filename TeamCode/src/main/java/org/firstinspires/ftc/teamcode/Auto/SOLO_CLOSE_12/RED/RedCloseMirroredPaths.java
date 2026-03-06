package org.firstinspires.ftc.teamcode.Auto.SOLO_CLOSE_12.RED;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class RedCloseMirroredPaths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain thirdIntake;
    public PathChain thirdReturn;

    public RedCloseMirroredPaths(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.293, 123.122).mirror(),
                                new Pose(30.866, 112.268).mirror(),
                                new Pose(38.854, 104.537).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(45))

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.805, 96.537).mirror(),
                                new Pose(22.434, 100.071).mirror(),
                                new Pose(21.078, 94.899).mirror(),
                                new Pose(15.098, 89.878).mirror(),
                                new Pose(13.610, 86.488).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.610, 86.488).mirror(),

                                new Pose(51.805, 89.488).mirror()
                        )
                ).setLinearHeadingInterpolation(0, Math.toRadians(45))

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(39.854, 104.537).mirror(),
                                new Pose(65.683, 69.878).mirror(),
                                new Pose(35.720, 61.890).mirror()

                        )
                ).setTangentHeadingInterpolation()

                .addPath(
                        new BezierLine(

                                new Pose(35.720, 61.890).mirror(),
                                new Pose(16.000, 57.707).mirror()
                        )).setConstantHeadingInterpolation(0)

                .addPath(
                        new BezierCurve(
                                new Pose(12, 57.707).mirror(),
                                new Pose(18, 55.707).mirror(),
                                new Pose(19, 65.707).mirror(),
                                new Pose(15, 65.707).mirror()

                        )).setConstantHeadingInterpolation(0)

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.000, 59.707).mirror(),

                                new Pose(30, 60).mirror(),
                                new Pose(40, 80).mirror(),

                                new Pose(47.488, 97.829).mirror()
                        )

                ).setReversed()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.25,
                                        HeadingInterpolator.constant(0)

                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.25,
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


        thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.805, 89.488).mirror(),
                                new Pose(24.451, 59.207).mirror(),
                                new Pose(21.549, 55.183).mirror(),
                                new Pose(20.098, 31.951).mirror()
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(270))
                                )
                        )
                )

                .build();

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.829, 35.951).mirror(),

                                new Pose(49.146, 102.683).mirror()
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.8,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(36))
                                )
                        )
                )

                .build();
    }
}
