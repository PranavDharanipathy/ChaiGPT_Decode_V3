package org.firstinspires.ftc.teamcode.Auto.Alliance.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.MirroredPose;

public class RedAllianceClosePaths {
    public PathChain preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain thirdIntake;
    public PathChain thirdIntakeGate;
    public PathChain thirdReturn;

    public RedAllianceClosePaths(Follower follower) {
        preload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(20.293, 123.122),
                                new MirroredPose(30.866, 112.268),
                                new MirroredPose(50, 88)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), 0)

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(53, 100),
                                new MirroredPose(24.434, 110.071),
                                new MirroredPose(23.078, 110.899),
                                new MirroredPose(23.098, 91.878),
                                new MirroredPose(21.610, 86.488)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
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

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(14.5, 75),

                                new MirroredPose(52, 89.488)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.2,
                                        HeadingInterpolator.constant(Math.toRadians(255))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.2,
                                        1,
                                        HeadingInterpolator.linear(Math.toRadians(255), Math.toRadians(-45))
                                )
                        )
                )

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(52, 88),
                                new MirroredPose(60.683, 69.878),
                                new MirroredPose(30.720, 61.890)

                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.28,
                                        HeadingInterpolator.constant(Math.toRadians(-45))
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
                                new MirroredPose(30.720, 61.890),
                                new MirroredPose(10.5, 57.707)
                        )).setConstantHeadingInterpolation(0)

                .addPath(
                        new BezierCurve(
                                new MirroredPose(10.5, 57.707),
                                new MirroredPose(18, 55.707),
                                new MirroredPose(19, 65.707),
                                new MirroredPose(14.25, 65.707)

                        )).setConstantHeadingInterpolation(0)

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new MirroredPose(14.25, 59.707),
                                new MirroredPose(53, 100)
                        )
                )
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
                                        HeadingInterpolator.facingPoint(new MirroredPose(14, 59.707).getPose())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(-60))
                                )
                        )
                )

                .build();

        thirdIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new MirroredPose(58.805, 89.488),
                                new MirroredPose(24.451, 59.207),
                                new MirroredPose(22, 55.183),
                                new MirroredPose(20, 35.951)
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
                                        HeadingInterpolator.constant(Math.toRadians(-90))
                                )
                        )
                )

                .build();

        thirdIntakeGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new MirroredPose(20, 35.951),
                                new MirroredPose(26, 50),
                                new MirroredPose(25, 65),
                                new MirroredPose(13.5, 68)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))

                .build();

        thirdReturn = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new MirroredPose(13.5, 68),
                                new MirroredPose(53, 103.683)
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
                                        HeadingInterpolator.facingPoint(new MirroredPose(14.5, 68).getPose())
                                )
                        )
                )

                .build();
    }
}