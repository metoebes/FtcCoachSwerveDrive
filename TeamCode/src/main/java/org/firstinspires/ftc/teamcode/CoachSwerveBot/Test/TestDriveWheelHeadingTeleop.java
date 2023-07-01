package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test;

import android.os.Environment;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Config.ConfigDriveSpeed;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.AbstractTestCase;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Config.ConfigTurnSpeed;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.Pivot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.Straffe;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.SweepCCW;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.SweepCW;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

/* graph radians/angle, driveMotorDirection, target Encoder Count, current encoder count */
@TeleOp(name = "TestDriveWheelHeading", group = "coach")
public class TestDriveWheelHeadingTeleop extends OpMode {
    final public static int CHOOSE_TEST = 0;
    final public static String CONFIG_FOLDER = "config";
    final public static String CONFIG_FILENAME = "test";

    Robot robot = null;
    ImprovedGamepad impGamepad1 = null;
    public ElapsedTime gamepadTimer = new ElapsedTime();  // SECONDS is the default resolution for time() method

    Config config = null;
    TestMenu menu = null;
    double angle = 0;
    double radius = 0;

    int action = CHOOSE_TEST;
    int testCaseStep = 0;
    public DataLogger datalog;
    boolean completed = false;
    double driveSpeed;
    AbstractTestCase testCase = null;
    ArrayList<AbstractTestCase> testCases = new ArrayList();

    @Override
    public void init() {
        // cd C:\Users\metoe\nodejsProjects\ftc-dashboard\FtcDashboard\dash
        // yarn start
        // connect to robot controller wifi
        // open browser:  http://192.168.43.1:8080/dash
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1");

        config = readConfigFile(CONFIG_FILENAME);

        robot = new Robot();
        robot.init(this.hardwareMap, config, telemetry);

        menu = new TestMenu(impGamepad1, telemetry);

        testCase = new SweepCCW(robot);
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);

        testCase = new SweepCW(robot);
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);

        testCase =new Pivot(robot);
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);

        testCase = new Straffe(robot,impGamepad1, telemetry);
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);

        testCase = new ConfigTurnSpeed(robot,config, impGamepad1, telemetry);
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);

        testCase = new ConfigDriveSpeed(robot,config, impGamepad1, telemetry);
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);

        testCase = null;
    }

    public Config readConfigFile(String filename) {
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

    public void writeConfig(String filename) {
        final File configDir = new File(Environment.getExternalStorageDirectory(), CONFIG_FOLDER);
        final File file = new File(configDir, filename);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            ObjectMapper mapper = new ObjectMapper();
            String jsonString = mapper.writerWithDefaultPrettyPrinter().writeValueAsString(this.config);
            writer.write(jsonString);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
        completed = false;
        impGamepad1.update();

        if (testCase == null) {
            menu.update();
            if (menu.isItemSelected()) {
                testCase = testCases.get(menu.getSelectedItemIndex());
                testCase.init();
            }
        } else {
            testCase.update();
            if (testCase.isComplete()) {
                testCase = null;
                writeConfig(CONFIG_FILENAME);
            }
        }
    }
}

