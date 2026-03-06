package org.firstinspires.ftc.teamcode.Auto.SOLO_CLOSE_12.BLUE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class Close12Paths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain thirdIntake;
    public PathChain thirdReturn;

    public Close12Paths(Follower follower) {
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
                                new Pose(56.488, 83.829),
                                new Pose(24.434, 110.071),
                                new Pose(23.078, 110.899),
                                new Pose(23.098, 91.878),
                                new Pose(21.610, 86.488)
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
                        new Pose(12.000, 57.707)
                )).setConstantHeadingInterpolation(Math.PI)

                .addPath(
                        new BezierCurve(
                                new Pose(12, 57.707),
                                new Pose(18, 55.707),
                                new Pose(19, 65.707),
                                new Pose(15, 65.707)

                        )).setConstantHeadingInterpolation(Math.PI)

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.000, 59.707),

                                new Pose(56.488, 83.829)
                        )

                ).setReversed()
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
