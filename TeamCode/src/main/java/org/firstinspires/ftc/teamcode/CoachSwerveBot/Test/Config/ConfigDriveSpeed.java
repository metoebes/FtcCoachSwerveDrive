package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.AbstractTestCase;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;


public class ConfigDriveSpeed extends AbstractTestCase {
    double radius;
    private final ImprovedGamepad impGamepad1;
    private final Telemetry telemetry;
    Config config;

    public ConfigDriveSpeed(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        this.impGamepad1 = _impGamepad1;
        this.telemetry = _telemetry;
        this.robot = _robot;
        this.config = _config;
        testCaseName = "Set Drive Speed";
    }
    private void help() {
        telemetry.addData("L/R bumper", "-/+ turn speed");
        telemetry.addData("   Drive Speed", config.driveSpeed);
        telemetry.addData("Press X", "to exit " + testCaseName);
    }
    public boolean update() {
        help();
        switch (testCaseStep) {
            case 0: {
                testCaseStep++;
                break;
            }
            case 1: {
                // right bumper -  Increase Turn Speed
                if (impGamepad1.y.isInitialPress()) {
                    config.driveSpeed += .1;
                    if (config.driveSpeed > 1)
                        config.driveSpeed = 1;
                }
                // left bumper -Decrease Turn Speed
                else if (impGamepad1.a.isInitialPress()) {
                    config.driveSpeed -= .1;
                    if (config.driveSpeed < 0)
                        config.driveSpeed = 0;
                }
                else if (impGamepad1.x.isInitialPress()) {
                    testCaseStep++;
                }
                break;
            }
            case 2: {
                break;
            }
        }
        telemetry.update();
        return isComplete();
    }
    public boolean isComplete() {
        return testCaseStep == 2;
    }
}
