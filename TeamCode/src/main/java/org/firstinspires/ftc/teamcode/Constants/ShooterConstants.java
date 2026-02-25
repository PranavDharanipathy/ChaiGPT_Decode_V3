package org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.List;

public class ShooterConstants {

    public static double TURRET_TICKS_PER_DEGREE = 91.0222222222;

    public static List<Double> TURRET_PD_POSITIONS = new ArrayList<>(List.of(-11000.0,  -10000.0,    -9000.0,   -8000.0,   -7000.0,   -6000.0,    -5000.0,    -4000.0,    -3000.0,   -2000.0,  -1000.0,     0.0,      1000.0,    2000.0,    3000.0,   4000.0,   5000.0,   6000.0,    7000.0,    8000.0,   9000.0,   10000.0,   11000.0,    12000.0,   13000.0));
    public static List<Double> TURRET_KPS =          new ArrayList<>(List.of(0.000285,   0.000279,   0.000285,   0.0002,    0.0002,   0.00014,    0.00011,    0.00009,    0.00008,   0.00007,  0.00005,   0.00005,   0.000092,  0.0000975,  0.0001,   0.00011,  0.00012,  0.00012,  0.00012,    0.00011,  0.00011,  0.0001,    0.00014,    0.0002,    0.00033));
    public static List<Double> TURRET_KDS =          new ArrayList<>(List.of(  0.003,    0.00318,     0.003,      0.003,    0.003,     0.0028,     0.0024,     0.0022,     0.0022,    0.002,    0.002,     0.002,     0.00385,   0.00375,   0.0032,   0.00353,  0.0035,   0.0032,    0.0032,     0.003,    0.003,   0.0035,    0.0031,     0.0025,     0.002));

    public static List<Double> TURRET_FEEDFORWARD_POSITIONS = new ArrayList<>(List.of(-110000.0, -10000.0,   -9000.0,    -8000.0,    -7000.0,    -6000.0,    -5000.0,     -4000.0,    -3000.0,   -2000.0,    -1000.0,     0.0,       1000.0,    2000.0,    3000.0,     4000.0,     5000.0,   6000.0,     7000.0,     8000.0,   9000.0,    10000.0,   11000.0,    12000.0,   13000.0));
    public static List<Double> TURRET_KFS =                   new ArrayList<>(List.of( 0.000003, 0.0000021,  0.0000021,  0.000002,  0.00000175, 0.00000185,  0.0000018,  0.00000185, 0.0000025, 0.0000025,   0.000003,  0.000011,   0.000008,  0.000007,  0.0000032,   0.000003,  0.000003, 0.000003,  0.0000031,  0.0000029, 0.000003,  0.00000305, 0.0000035,  0.000004,  0.000006));

    public static double TURRET_KF_RESISTANCE_ENGAGE_ERROR = 2000;

    public static double TURRET_HOME_POSITION_INCREMENT = 150;

    public static double TURRET_POSITIONAL_OFFSET = ;
    public static double TURRET_ANGULAR_OFFSET = 0;

}
