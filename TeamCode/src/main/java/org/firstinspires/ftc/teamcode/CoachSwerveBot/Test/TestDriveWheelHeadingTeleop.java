package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.AbstractTestCase;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.ConfigTestCase;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.Pivot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.Straffe;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.SweepCCW;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.SweepCW;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.util.ArrayList;

/* graph radians/angle, driveMotorDirection, target Encoder Count, current encoder count */
@TeleOp(name = "TestDriveWheelHeading", group = "coach")
public class TestDriveWheelHeadingTeleop extends OpMode {
    final public static int CHOOSE_TEST = 0;

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

        config = new Config();

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

        testCase = new ConfigTestCase(robot,config, impGamepad1, telemetry);
        menu.addItem(testCase.testCaseName);
        testCases.add(testCase);

        testCase = null;
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
            }
        }
    }
}

