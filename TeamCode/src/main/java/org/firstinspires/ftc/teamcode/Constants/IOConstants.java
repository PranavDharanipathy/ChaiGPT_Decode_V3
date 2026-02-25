package org.firstinspires.ftc.teamcode.Constants;

import org.firstinspires.ftc.teamcode.data.EOAOffset;

import java.util.HashMap;
import java.util.Map;

public class IOConstants {

    // End-of-auto (EOA) pose data file name
    public static String EOA_LOCALIZATION_DATA_FILE_NAME = "localizationData.csv";
    public static String EOA_LOCALIZATION_DATA_DELIMITER = ",";

    public static Map<String, EOAOffset> EOA_OFFSETS = new HashMap<>(
            Map.of("auto12", new EOAOffset(17.288, -32.73))
    );
}
