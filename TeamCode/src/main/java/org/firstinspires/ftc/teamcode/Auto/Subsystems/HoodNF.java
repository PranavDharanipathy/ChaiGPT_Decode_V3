package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import android.content.res.Configuration;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Systems.HoodAngler;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class HoodNF implements Subsystem {

    private HoodNF() {}

    public static final HoodNF INSTANCE = new HoodNF();

    public HoodAngler hood;

    @Override
    public void initialize() {
    hood = new HoodAngler(ActiveOpMode.hardwareMap(), "left_hood", "right_hood");
    hood.setServoDirections(ConfigurationConstants.HOOD_ANGLER_SERVO_DIRECTIONS);
        hood.setPosition(0.4); //init position
    }

    public Command setPosition(double position) {
        return new InstantCommand(() -> hood.setPosition(position));
    }

    @Override
    public void periodic() {

    }
}
