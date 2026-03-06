package org.firstinspires.ftc.teamcode.Auto.Close15.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class BlueClosePaths {
    public PathChain preload;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain gate;
    public PathChain gateReturn;
    public PathChain firstIntake;
    public PathChain firstReturn;
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

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(38.854, 104.537),
                                new Pose(60.683, 68.878),
                                new Pose(34.427, 69.451),
                                new Pose(48.329, 58.305),
                                new Pose(25.939, 60.573)
                                //new Pose(16.366, 61.439)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.5,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.5,
                                        1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
                        )

                )

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(25.366, 60.439),
                                new Pose(47.268, 63.805),
                                new Pose(52.220, 70.463),
                                new Pose(64.000, 84.098)
                        )

                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.facingPoint(9, 75)
                                        ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(135))
                                )
                        )
                )

                .build();

        /*gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.488, 87.829),
                                new Pose(53.829, 61.720),
                                new Pose(25.829, 52.207),
                                new Pose(16.463, 64.220),
                                new Pose(12.463, 64.220)
                        )


                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        0.8,
                                        HeadingInterpolator.constant(Math.toRadians(158))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(163))
                                )


                )).addPath(
                        new BezierCurve(
                                new Pose(12.463, 64.220),
                                new Pose(14.463, 60.220),
                                new Pose(14.463, 60.220),
                                new Pose(14.463, 60.220)
                        )
                ).setConstantHeadingInterpolation(Math.PI)

                .addPath(
                        new BezierLine(
                                new Pose(14.463, 60.220),
                                new Pose(10.463,60.220 )
                        )
                ).setConstantHeadingInterpolation(Math.PI)

                .build();   */


        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.488, 87.829),
                                new Pose(53.829, 61.720),
                                new Pose(25.829, 52.207),
                                new Pose(16.463, 60.220),
                                new Pose(12.463, 60.220)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        0.8,
                                        HeadingInterpolator.constant(Math.toRadians(158))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(163))
                                )
                        )
                )

                .build();


        gateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.463, 60.220),

                                new Pose(52.488, 87.829)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(62.366, 88.463),
                                new Pose(42.239, 83.095),
                                new Pose(19.122, 84.293)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.610, 84.488),

                                new Pose(58.805, 89.488)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.constant(Math.PI)
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
                                new Pose(58.805, 89.488),
                                new Pose(71.524, 40.854),
                                new Pose(36.427, 26.927),
                                new Pose(33.988, 39.000),
                                new Pose(10.829, 38.341)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.829, 36.341),

                                new Pose(50.146, 87.683)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.constant(Math.PI)
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
  