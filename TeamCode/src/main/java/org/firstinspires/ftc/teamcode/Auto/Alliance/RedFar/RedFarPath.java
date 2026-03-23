package org.firstinspires.ftc.teamcode.Auto.Alliance.RedFar;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class RedFarPath {
    public PathChain firstIntake;

    public PathChain firstReturn;

    public PathChain hpIntake;
    public PathChain hpReturn;
    public RedFarPath(Follower follower) {
        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(63.610, 6.829),
                                new Pose(55.805, 22.244),
                                new Pose(47.537, 35.780),
                                new Pose(39.220, 35.634),
                                new Pose(18.366, 35.220)
                        )
                ).setTangentHeadingInterpolation()

                .setGlobalDeceleration()

                .build();


        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.366, 35.220),

                                new Pose(55.366, 12.537)
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.tangent.reverse()


                                        ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
                        )
                )

                .build();


        hpIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.585, 12.317),

                                new Pose(8.780, 12.805)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        hpReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.780, 12.805),

                                new Pose(54.610, 12.293)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }
}