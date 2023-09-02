package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;

import javax.tools.Diagnostic;

public abstract class AbstractTestCase {
    public DataLogger datalog = null;
    public int testCaseStep = 0;
    public String testCaseName = "";
    public ElapsedTime timer = new ElapsedTime();
    public Robot robot = null;
    public ImprovedGamepad impGamepad1;
    public Telemetry telemetry;
    public Config config;

    public AbstractTestCase(String _testCaseName, Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        this.impGamepad1 = _impGamepad1;
        this.telemetry = _telemetry;
        this.testCaseName = _testCaseName;
        this.robot = _robot;
        this.config = _config;
    }

    public void startLogging(String name) {
        try {
            datalog = new DataLogger(name + ".txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void stoplogging() {
        if (datalog!=null) {
            datalog.close();
            datalog = null;
        }
    }
    public boolean update() {
        return true;
    }
    public boolean isComplete() {
        return true;
    }
    public void init() {testCaseStep=0;}

}
