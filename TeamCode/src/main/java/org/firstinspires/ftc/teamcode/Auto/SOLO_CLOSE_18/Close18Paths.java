package org.firstinspires.ftc.teamcode.Auto.SOLO_CLOSE_18;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class Close18Paths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain thirdIntake;
    public PathChain thirdReturn;

    public PathChain gate;

    public PathChain gateReturn;

    public Close18Paths(Follower follower) {
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
                                new Pose(21.610, 86.488),

                                new Pose(58.805, 89.488)
                        )
                ).setLinearHeadingInterpolation(Math.PI, Math.toRadians(141))

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(38.854, 104.537),
                                new Pose(60.683, 69.878),
                                new Pose(28.720, 61.890)

                        )
                ).setTangentHeadingInterpolation()

                .addPath(
                        new BezierLine(

                                new Pose(28.720, 61.890),
                                new Pose(16.000, 60.707)
                        )).setConstantHeadingInterpolation(Math.PI)

                .addPath(
                        new BezierCurve(
                                new Pose(16, 60.707),
                                new Pose(20, 52.707),
                                new Pose(22, 64.707),
                                new Pose(16, 65.707)

                        )).setConstantHeadingInterpolation(Math.toRadians(185))

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 65.707),



                                new Pose(64.488, 83.829)
                        )

                ).setReversed()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.76,
                                        HeadingInterpolator.tangent.reverse()

                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.76,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(142))
                                )
                        )
                )

                .build();


        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(64.488, 83.829),
                                new Pose(53.829, 60.720),
                                new Pose(25.829, 62.207),
                                new Pose(12.463, 62.220)
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
                                        HeadingInterpolator.constant(Math.toRadians(159))
                                )
                        )
                )

                .build();

        gateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.463, 62.220),

                                new Pose(64.488, 86.829)
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
                                        HeadingInterpolator.constant(Math.toRadians(148))
                                )
                        )
                )

                .build();

        /*thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.488, 94.829),
                                new Pose(71.524, 40.854),
                                new Pose(36.427, 26.927),
                                new Pose(33.988, 39.000),
                                new Pose(18.829, 36.341)
                        )
                ).setTangentHeadingInterpolation()

                .build(); */

        thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.805, 89.488),
                                new Pose(44.451, 59.207),
                                new Pose(34.549, 55.183),
                                new Pose(26.098, 35.951)
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

        /*

        port 0 - SErvo(Axon) - spin axon for 1 second
        port 2 - Spin a CRServo with power 1
         */

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.829, 35.951),

                                new Pose(59.146, 103.683)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(150))
                                )
                        )
                )

                .build();
    }
}
