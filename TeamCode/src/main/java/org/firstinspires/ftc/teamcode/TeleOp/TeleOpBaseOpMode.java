package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.HoodAngler;
import org.firstinspires.ftc.teamcode.Systems.TurretBase;
import org.firstinspires.ftc.teamcode.data.EOALocalization;
import org.firstinspires.ftc.teamcode.data.LocalizationData;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.GeneralVeloMotor;

import java.util.List;

public abstract class TeleOpBaseOpMode extends OpMode {

    private boolean localizationFromAuto = false;
    private LocalizationData localizationData;

    public void useEOALocalizationData() {

        localizationData = EOALocalization.read();
        //localizationData = (LocalizationData) blackboard.get("EOALocalization");

        localizationFromAuto = true;
    }

    public TeleOpBaseOpMode() {}

    public BetterGamepad controller1;
    public BetterGamepad controller2;

    public DcMotor left_front, right_front, left_back, right_back;

    public Rev9AxisImu rev9AxisImu;

    public GeneralVeloMotor transfer;

    public DcMotor intake;

    public Camera unstartedCamera;

    public Follower follower;

    public TurretBase turret;

    public HoodAngler hoodAngler;

    public Flywheel flywheel;

    private List<LynxModule> robotHubs;

    /// initializes/creates LynxModule
    public void setUpLynxModule() {

        robotHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : robotHubs) {
            hub.setBulkCachingMode(MapSetterConstants.bulkCachingMode);
        }
    }

    /// Clears cache of LynxModule
    public void clearCacheOfLynxModule() {

        for (LynxModule hub : robotHubs) {
            hub.clearBulkCache();
        }
    }

    /// Closes LynxModule
    public void closeLynxModule() {

        for (LynxModule hub : robotHubs) {
            hub.close();
        }
    }

    /// Initializing devices
    public void initializeDevices() {

        left_front = hardwareMap.get(DcMotor.class, MapSetterConstants.leftFrontMotorDeviceName);
        right_front = hardwareMap.get(DcMotor.class, MapSetterConstants.rightFrontMotorDeviceName);
        left_back = hardwareMap.get(DcMotor.class, MapSetterConstants.leftBackMotorDeviceName);
        right_back = hardwareMap.get(DcMotor.class, MapSetterConstants.rightBackMotorDeviceName);

        follower = DriveConstants.createTeleOpFollower(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, MapSetterConstants.intakeMotorDeviceName);

        transfer = new GeneralVeloMotor(hardwareMap, MapSetterConstants.transferMotorDeviceName);

        unstartedCamera = new Camera(follower, Camera.from(hardwareMap, MapSetterConstants.limelight3AUSBDeviceName));

        flywheel = new Flywheel(
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        if (localizationFromAuto) {
            turret = new TurretBase(hardwareMap, localizationData.getTurretStartPosition());
        }
        else {
            turret = new TurretBase(hardwareMap);
        }

        hoodAngler = new HoodAngler(hardwareMap,
                MapSetterConstants.hoodAnglerLeftServoDeviceName,
                MapSetterConstants.hoodAnglerRightServoDeviceName
        );

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    /// Provide traits
    public void applyComponentTraits() {

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (localizationFromAuto) {
            follower.setPose(localizationData.getPose());
        }
        else {
            follower.setPose(FieldConstants);
        }

        follower.update();

        transfer.setDirection(DcMotor.Direction.REVERSE);

        transfer.setVelocityPDFCoefficients(
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[0],
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[1],
                ConfigurationConstants.TRANSFER_PDF_COEFFICIENTS[2]
        );

        unstartedCamera.setPollRateHz(CameraConstants.CAMERA_POLL_RATE);

        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setInternalParameters(
                ShooterInformation.ShooterConstants.getTotalFlywheelAssemblyWeight(),
                ShooterInformation.ShooterConstants.SHAFT_DIAMETER,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_RPM
        );
        flywheel.setVelocityPIDVSCoefficients(
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[0],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[1],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[2],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[3],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[4],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[5],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[6],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[7],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[8]
        );
        flywheel.setVoltageFilterAlpha(Constants.FLYWHEEL_VOLTAGE_FILTER_ALPHA);
        flywheel.setIConstraints(Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT);
        flywheel.setPConstraints(Constants.FLYWHEEL_MIN_PROPORTIONAL_LIMIT, Constants.FLYWHEEL_MAX_PROPORTIONAL_LIMIT);

        turret.setPIDFSCoefficients(Constants.TURRET_PIDFS_COEFFICIENTS);
        turret.reverse();

        //clear
        EOALocalization.blank();
    }

}