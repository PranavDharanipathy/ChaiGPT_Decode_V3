package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Constants.MapSetterConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;


public class FlywheelNF implements Subsystem {


    public FlywheelNF() {}

    public static FlywheelNF INSTANCE = new FlywheelNF();
    public Flywheel flywheel;

    @Override

    public void initialize() {


        flywheel = new Flywheel(
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, MapSetterConstants.leftFlywheelMotorDeviceName),
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        flywheel.setInternalParameters(
                ConfigurationConstants.FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT,
                ConfigurationConstants.FLYWHEEL_SHAFT_DIAMETER,
                ConfigurationConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ConfigurationConstants.FLYWHEEL_MOTOR_RPM
        );

        flywheel.initVoltageSensor(ActiveOpMode.hardwareMap());
        flywheel.setVelocityPIDVSCoefficients(ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS);

        flywheel.reset();


    }

    public Command setVel(double vel, boolean allowIntegralReset) {
        return new InstantCommand(() -> flywheel.setVelocity(vel, allowIntegralReset));
    }

    public Command setVel(double vel) {
        return new InstantCommand(() -> flywheel.setVelocity(vel, true));
    }

    public void end() {
        flywheel.setVelocity(0, true);
    }

    @Override
    public void periodic() {

        flywheel.update();
    }
}
