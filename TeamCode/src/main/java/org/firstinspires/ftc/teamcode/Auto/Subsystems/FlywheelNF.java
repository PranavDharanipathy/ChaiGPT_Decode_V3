package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
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
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "left_flywheel"),
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, "right_flywheel")
        );

        flywheel.setInternalParameters(
                ConfigurationConstants.FLYWHEEL_ASSEMBLY_TOTAL_WEIGHT,
                ConfigurationConstants.FLYWHEEL_SHAFT_DIAMETER,
                ConfigurationConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ConfigurationConstants.FLYWHEEL_MOTOR_RPM
        );

        flywheel.initVoltageSensor(ActiveOpMode.hardwareMap());


        // NO FUNCTION FOUND CALLED SETVOLTAGE FILTER ALPHA
        //flywheel.setVoltageFilterAlpha(FlywheelDriveTuning.VOLTAGE_FILTER_ALPHA);
        flywheel.setVelocityPIDVSCoefficients(ConfigurationConstants.FLYWHEEL_PIDVS_COEFFICIENTS);


        //NO FUNCTION FOUND TO SET I CONSTRAINTS.
        //flywheel.setIConstraints(Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT);

        flywheel.reset();


    }

    public Command setVel(double vel, boolean allowIntegralReset) {
        return new InstantCommand(() -> flywheel.setVelocity(vel, allowIntegralReset));
    }

    private double velWanted, velInflated, velSwitch;
    private boolean useVelCatching = false;

    public void setVelCatch(double m_velWanted, double m_velInflated, double m_velSwitch) {

        useVelCatching = true;
        velWanted = m_velWanted;
        velInflated = m_velInflated;
        velSwitch = m_velSwitch;
    }

    public Command setVelCatchCmd(double m_velWanted, double m_velInflated, double m_velSwitch) {

        return new Command() {

            @Override
            public boolean isDone() {

                useVelCatching = true;
                velWanted = m_velWanted;
                velInflated = m_velInflated;
                velSwitch = m_velSwitch;

                return true;
            }
        };
    }

    public void end() {
        flywheel.setVelocity(0, true);
    }

    @Override
    public void periodic() {

        if (useVelCatching) {

            if (Math.abs(flywheel.getTargetVelocity() - flywheel.getCurrentVelocity()) > velSwitch) {
                flywheel.setVelocity(velInflated, false);
            }
            else {
                flywheel.setVelocity(velWanted, false);
            }
        }

        //flywheel.updateKvBasedOnVoltage();
        flywheel.update();
    }
}
