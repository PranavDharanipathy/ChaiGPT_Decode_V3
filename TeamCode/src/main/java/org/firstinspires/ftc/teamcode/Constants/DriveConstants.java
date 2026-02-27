package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class DriveConstants {

    public static FollowerConstants followerConstants = new FollowerConstants()

            .mass(13.19954)

            .forwardZeroPowerAcceleration(66.62530181163879)
            .lateralZeroPowerAcceleration(-51.82186721441313)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0, 0.018, 0.175))

            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.055,0.1))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.0018,0.85,0.1))

            .translationalPIDFSwitch(3)
            .headingPIDFSwitch(0.17453299)
            .drivePIDFSwitch(20)

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.003,0.035))

            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.1,0.0005,0,0.025))

            .centripetalScaling(0)
            ;

    public static PathConstraints autoPathConstraints = new PathConstraints(0.994, 50, 1.5, 1.5);
    public static PathConstraints teleOpPathConstraints = new PathConstraints(0.994, 50, 1.6, 1.5);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(MapSetterConstants.rightFrontMotorDeviceName)
            .rightRearMotorName(MapSetterConstants.rightBackMotorDeviceName)
            .leftRearMotorName(MapSetterConstants.leftBackMotorDeviceName)
            .leftFrontMotorName(MapSetterConstants.leftFrontMotorDeviceName)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(-41.2745147944275)
            .yVelocity(-61.15854729822397);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.83464)
            .strafePodX(5.86613)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(MapSetterConstants.pinpointOdometryComputerDeviceName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createAutoFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(autoPathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static Follower createTeleOpFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(teleOpPathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
