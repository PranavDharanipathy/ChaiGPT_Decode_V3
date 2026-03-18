package org.firstinspires.ftc.teamcode.Auto.SOLO_CLOSE_18;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.MirroredPose;

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
                )

                //.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(200) )

                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(230))
                                )
                        )
                )

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.488, 73.829),
                                new Pose(14.5, 92)


                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                /*.addPath(
                        new BezierCurve(
                                new MirroredPose(21.610, 86.488),
                                new MirroredPose(25, 84.5),
                                new MirroredPose(28.5, 75)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        new BezierLine(
                                new MirroredPose(28.5, 75),
                                new MirroredPose(14.5, 75)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
 */
                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.610, 86.488),

                                new Pose(48.805, 89.488)
                        )
                ).setLinearHeadingInterpolation(Math.PI, Math.toRadians(170))

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52, 88),
                                new Pose(60.683, 69.878),
                                new Pose(17.720, 61.890)

                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.28,
                                        HeadingInterpolator.constant(Math.toRadians(180+45))
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
                                new Pose(18.5, 57.707)
                        )).setConstantHeadingInterpolation(0)

                /*.addPath(
                        new BezierCurve(
                                new MirroredPose(18.5, 57.707),
                                new MirroredPose(18, 55.707),
                                new MirroredPose(19, 65.707),
                                new MirroredPose(14.25, 65.707)

                        )).setConstantHeadingInterpolation(0) */

                .build();
        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.5, 57.707),



                                new Pose(58.488, 82.829)
                        )

                ).setReversed()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.86,
                                        HeadingInterpolator.tangent.reverse()

                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.86,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(180))
                                )
                        )
                )

                .build();


        /*gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.488, 78.829),
                                new Pose(42.829, 67.720),
                                new Pose(26.829, 64.207),
                                new Pose(12.6, 64.220)
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
                                        HeadingInterpolator.constant(Math.toRadians(173))
                                )
                        )
                )




                .build();*/

        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.488, 80.829),
                                new Pose(42.829, 69.720),
                                new Pose(22.829, 64.207),
                                new Pose(13, 63.220)
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
                                        HeadingInterpolator.constant(Math.toRadians(162))
                                )
                        )
                )




                .build();

        gateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.463, 64.220),

                                new Pose(58.488, 74.829)
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
                                        HeadingInterpolator.constant(Math.toRadians(190))
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
                                new Pose(36.451, 59.207),
                                new Pose(23.549, 59.183),
                                new Pose(21.098, 35.951)
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
                                new Pose(28.829, 35.951),

                                new Pose(53.146, 81.683)
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
                                        HeadingInterpolator.constant(Math.toRadians(165))
                                )
                        )
                )

                .build();
    }
}
