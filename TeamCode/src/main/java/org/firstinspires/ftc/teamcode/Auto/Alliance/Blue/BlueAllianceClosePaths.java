package org.firstinspires.ftc.teamcode.Auto.Alliance.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class BlueAllianceClosePaths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain thirdIntake;
    public PathChain thirdIntakeGate;
    public PathChain thirdReturn;

    public BlueAllianceClosePaths(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.293, 123.122),
                                new Pose(30.866, 112.268),
                                new Pose(50, 88)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.PI)

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58, 95),
                                new Pose(24.434, 110.071),
                                new Pose(23.078, 110.899),
                                new Pose(23.098, 91.878),
                                new Pose(21.610, 86.488)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        new BezierCurve(
                            new Pose(21.610, 86.488),
                            new Pose(25, 84.5),
                            new Pose(28.5, 75)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        new BezierLine(
                                new Pose(28.5, 75),
                                new Pose(13, 75)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13, 75),

                                new Pose(58.805, 89.488)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.2,
                                        HeadingInterpolator.constant(Math.toRadians(-75))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.2,
                                        1,
                                        HeadingInterpolator.linear(Math.toRadians(-75), Math.toRadians(225))
                                )
                        )
                )

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50, 88),
                                new Pose(60.683, 69.878),
                                new Pose(30.720, 61.890)

                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.28,
                                        HeadingInterpolator.constant(Math.toRadians(225))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.28,
                                        1,
                                        HeadingInterpolator.tangent
                                )
                        )
                )
                .addPath(
                        new BezierLine(
                                new Pose(30.720, 61.890),
                                new Pose(10.5, 57.707)
                        )).setConstantHeadingInterpolation(Math.PI)

                .addPath(
                        new BezierCurve(
                                new Pose(10.5, 57.707),
                                new Pose(18, 55.707),
                                new Pose(19, 65.707),
                                new Pose(15.7, 65.707)

                        )).setConstantHeadingInterpolation(Math.PI)

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.7, 59.707),
                                new Pose(58, 95)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.25,
                                        HeadingInterpolator.constant(Math.PI)

                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.25,
                                        0.6,
                                        HeadingInterpolator.facingPoint(new Pose(13, 59.707))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(240))
                                )
                        )
                )

                .build();

        thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.805, 89.488),
                                new Pose(24.451, 59.207),
                                new Pose(23, 55.183),
                                new Pose(23, 35.951)
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

        thirdIntakeGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(23, 35.951),
                                new Pose(26, 50),
                                new Pose(25, 65),
                                new Pose(13.5, 68)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))

                .build();

        thirdReturn = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.5, 68),
                                new Pose(57, 103.683)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.67,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.67,
                                        1,
                                        HeadingInterpolator.facingPoint(new Pose(14.5, 68))
                                )
                        )
                )

                .build();
    }
}