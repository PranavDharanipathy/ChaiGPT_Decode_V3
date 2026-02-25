package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Calculations;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;

public class Camera {

    private final Follower follower;
    private final Limelight3A limelight;

    public Limelight3A ll() {
        return limelight;
    }

    public Camera(Follower follower, Limelight3A limelight) {

        this.follower = follower;
        this.limelight = limelight;

        setPollRateHz(CameraConstants.CAMERA_POLL_RATE);
    }

    public static Limelight3A from(HardwareMap hardwareMap, String limelightDeviceName) {
        return hardwareMap.get(Limelight3A.class, limelightDeviceName);
    }

    public void start() {
        limelight.start();
    }

    public void setPollRateHz(int hz) {
        limelight.setPollRateHz(hz);
    }

    public void pipelineSwitch(int index) {
        limelight.pipelineSwitch(index);
    }

    public void reloadPipeline() {
        limelight.reloadPipeline();
    }

    private Pose botPose;
    private double distanceToTag;

    public void update() {

        limelight.updateRobotOrientation();

        LLResult llResult = limelight.getLatestResult();

        if (limelight.isConnected() && llResult != null && llResult.isValid()) {

            distanceToTag = llResult.getBotposeAvgDist();

            botPose = Calculations.convertPose3DtoPedroPose(llResult.getBotpose_MT2());
        }
    }

    public Pose getBotPose() {
        return botPose;
    }

}
