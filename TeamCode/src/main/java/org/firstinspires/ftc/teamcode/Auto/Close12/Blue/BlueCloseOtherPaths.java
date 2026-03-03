package org.firstinspires.ftc.teamcode.Auto.Close12.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueCloseOtherPaths {
    public PathChain Preload;
    public PathChain firstIntake;
    public PathChain firstReturn;
    public PathChain secondIntake;
    public PathChain secondReturn;
    public PathChain line6;
    public PathChain thirdReturn;

    public BlueCloseOtherPaths(Follower follower) {
        Preload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.293, 123.122),
                                new Pose(30.145, 113.195),
                                new Pose(37.997, 105.387)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        firstIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.997, 105.387),
                                new Pose(29.122, 101.247),
                                new Pose(23.356, 96.280),
                                new Pose(23.762, 89.542),
                                new Pose(23.442, 80.119)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        firstReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.442, 80.119),

                                new Pose(52.502, 95.038)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        secondIntake = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.502, 95.038),
                                new Pose(60.683, 68.878),
                                new Pose(33.744, 60.058),
                                new Pose(48.329, 58.305),
                                new Pose(30.646, 60.167),
                                new Pose(12.299, 58.626),
                                new Pose(8.846, 62.759)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        secondReturn = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(8.846, 62.759),
                                new Pose(29.306, 64.680),
                                new Pose(41.589, 77.987),
                                new Pose(61.057, 85.252)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        line6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(61.057, 85.252),
                                new Pose(63.963, 42.198),
                                new Pose(43.148, 27.935),
                                new Pose(35.500, 38.496),
                                new Pose(11.165, 35.837)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        thirdReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.165, 35.837),

                                new Pose(56.699, 90.642)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}
  