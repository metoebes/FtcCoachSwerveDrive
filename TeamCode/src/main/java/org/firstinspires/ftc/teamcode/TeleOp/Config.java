package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import com.fasterxml.jackson.databind.ObjectMapper;

public class Config {
    final public static String CONFIG_FOLDER = "config";
    public static final float DEFAULT_SPEED_UP = (float).007;

    public float driveSpeed = (float) .5;
    public float driveSpeedIncrement = DEFAULT_SPEED_UP;
    public float turnSpeed = (float)  .5;
    public float turnSpeedIncrement = 2 * DEFAULT_SPEED_UP;
    public boolean useDataLogger  = false;

    public void init(String filename , Telemetry _telemetry ) {
        readConfigFile(filename, _telemetry);
    }

    public Config readConfigFile(String filename, Telemetry telemetry) {
        final File configDir = new File(Environment.getExternalStorageDirectory(), CONFIG_FOLDER);
        final File file = new File(configDir, filename);
        Config config = null;

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line = reader.readLine();
            telemetry.addData("config", line );

            ObjectMapper mapper = new ObjectMapper();
            config = mapper.readValue(line, Config.class);
            if  (config == null) {
                return new Config();
            }
            telemetry.addData("successfully created Config object", "");
            telemetry.update();

        } catch (Exception e) {
            config = new Config();
        }
        return config;
    }

}
