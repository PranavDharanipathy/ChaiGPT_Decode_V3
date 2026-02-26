package org.firstinspires.ftc.teamcode.data;

import static org.firstinspires.ftc.teamcode.Constants.IOConstants.EOA_LOCALIZATION_DATA_DELIMITER;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.IOConstants;

import java.io.File;

/// Used {@link ReadWriteFile}
/// Provides robot x, y, REV 9-Axis heading (radians), turret start position
public class EOALocalization {

    /// @param xOffset is added to output x value (in TeleOp format nomenclature)
    /// @param yOffset is added to output y value (in TeleOp format nomenclature)
    public static Pose autoFormatToTeleOpFormat(Pose autoPose, double xOffset, double yOffset) {
        return new Pose(autoPose.getX() - 72 + xOffset, autoPose.getY() - 72 + yOffset, autoPose.getHeading());
    }

    /// Write pose of 0,0,0 and turret start position of 0
    public static void blank() {
        write(new Pose(), 0);
    }

    public static void write(double x, double y, double headingRad, double turretStartPosition) {

        File file = AppUtil.getInstance().getSettingsFile(IOConstants.EOA_LOCALIZATION_DATA_FILE_NAME);

        String localizationData = x + EOA_LOCALIZATION_DATA_DELIMITER + y + EOA_LOCALIZATION_DATA_DELIMITER + headingRad;

        ReadWriteFile.writeFile(file, localizationData);
    }

    public static void write(Pose pose, double turretStartPosition) {

        File file = AppUtil.getInstance().getSettingsFile(IOConstants.EOA_LOCALIZATION_DATA_FILE_NAME);

        String localizationData = pose.getX() + EOA_LOCALIZATION_DATA_DELIMITER + pose.getY() + EOA_LOCALIZATION_DATA_DELIMITER + pose.getHeading() + EOA_LOCALIZATION_DATA_DELIMITER + turretStartPosition;

        ReadWriteFile.writeFile(file, localizationData);
    }

    public static LocalizationData read() {

        File file = AppUtil.getInstance().getSettingsFile(IOConstants.EOA_LOCALIZATION_DATA_FILE_NAME);

        if (!file.exists()) { //if file doesn't exist use default pose
            return new LocalizationData(FieldConstants.RELOCALIZATION_POSE, null);
        }

        try {

            String data = ReadWriteFile.readFile(file).trim();

            //splitting into x, y, and heading
            String[] dataComponents = data.split(EOA_LOCALIZATION_DATA_DELIMITER);

            if (dataComponents.length != 4) { //if we don't have (x, y, heading, turret start position)
                return new LocalizationData(FieldConstants.RELOCALIZATION_POSE, null);
            }

            double x = Double.parseDouble(dataComponents[0]);
            double y = Double.parseDouble(dataComponents[1]);
            double heading = Double.parseDouble(dataComponents[2]);
            double turretStartPosition = Double.parseDouble(dataComponents[3]);

            return new LocalizationData(new Pose(x, y, heading), turretStartPosition);
        }
        catch (Exception e) { //if file has trash data use default pose
            return new LocalizationData(FieldConstants.RELOCALIZATION_POSE, null);
        }

    }

}
