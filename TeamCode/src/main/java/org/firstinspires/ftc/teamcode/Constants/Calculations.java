package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Calculations {

    public static Pose convertPose3DtoPedroPose(Pose3D pose3d) {

        double x = MathUtil.metersToInches(pose3d.getPosition().x);
        double y = MathUtil.metersToInches(pose3d.getPosition().y);
        double yaw = pose3d.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(x, y, yaw).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
