package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test;

import android.os.Environment;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Config.ConfigDriveSpeed;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.AbstractTestCase;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Config.ConfigTurnSpeed;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.ChangeHeading;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.ChangeStraffeDirection;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

/* graph radians/angle, driveMotorDirection, target Encoder Count, current encoder count */
@TeleOp(name = "Swerve Drive v2", group = "coach")
public class TestSwerveDrive extends OpMode {
    final public static int CHOOSE_TEST = 0;
    final public static String CONFIG_FOLDER = "config";
    final public static String CONFIG_FILENAME = "test";

    Robot robot = null;
    ImprovedGamepad impGamepad1 = null;
    public ElapsedTime gamepadTimer = new ElapsedTime();  // SECONDS is the default resolution for time() method

    Config config = null;
    TestMenu menu = null;
    boolean completed = false;
    AbstractTestCase testCase = null;
    ArrayList<AbstractTestCase> testCases = new ArrayList();

    public AbstractTestCase addTest(AbstractTestCase testCase) {
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);
        return testCase;
    }
    @Override
    public void init() {
        // connect PC to robot controller wifi
        // open PC browser:  http://192.168.43.1:8080/dash
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1", 0.7f);

        config = readConfigFile(CONFIG_FILENAME);

        robot = new Robot();
        robot.init(this.hardwareMap, config, telemetry);

        menu = new TestMenu(impGamepad1, telemetry);

        testCase = addTest( new RobotCentricDrive(robot,config, impGamepad1, telemetry) );
        addTest( new ChangeHeading(robot,config, impGamepad1, telemetry)  );
        addTest( new ChangeStraffeDirection(robot,config, impGamepad1, telemetry)  );
        addTest( new ConfigTurnSpeed(robot,config, impGamepad1, telemetry) );
        addTest( new ConfigDriveSpeed(robot,config, impGamepad1, telemetry) );

        telemetry.addLine("Before Pressing Start:");
        telemetry.addLine("- Align drive wheels !!!! ");
        telemetry.addLine("- Gold is front right wheel ");
        telemetry.addLine("- Green is back left wheel");
        telemetry.addLine("- motors should not stick out");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("Source Code:");
        telemetry.addLine("https://github.com/metoebes/FtcCoachSwerveDrive");

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
                telemetry.clear();
            }
        }
    }
}

