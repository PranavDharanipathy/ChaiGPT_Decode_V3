package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import org.firstinspires.ftc.teamcode.Constants.ConfigurationConstants;
import org.firstinspires.ftc.teamcode.Systems.TurretBase;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class TurretNF implements Subsystem {


public TurretNF() {}

    public static final TurretNF INSTANCE = new TurretNF();

    public TurretBase turret;



    @Override
    public void initialize() {
        turret = new TurretBase(ActiveOpMode.hardwareMap());


        turret.setPIDFSCoefficients(ConfigurationConstants.TURRET_PIDFS_COEFFICIENTS);
        turret.reverse();
        goToHomePosition();

    }


    public Command setPosition(double position) {
        return new InstantCommand(() -> turret.setPosition(position + turret.startPosition));
    }

    ///  Tells turret to go to the start position
    public void goToHomePosition() {
        turret.setPosition(turret.startPosition);
    }

    public Command goToHomePositionCmd() {
        return new InstantCommand(() -> turret.setPosition(turret.startPosition));
    }

    @Override
    public void periodic() {

        turret.update();
    }
}


