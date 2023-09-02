package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.AbstractTestCase;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;


public class ConfigDriveSpeed extends AbstractTestCase {

    public ConfigDriveSpeed(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        super("Set Drive Speed", _robot, _config, _impGamepad1, _telemetry);
    }
    private void help() {
        telemetry.addData("A/Y Buttons", "-/+ turn speed");
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
                    telemetry.addData("increasing speed by .01","");
                    config.driveSpeed += .01;
                    if (config.driveSpeed > 1)
                        config.driveSpeed = 1;
                    telemetry.addData("speed is now ",config.driveSpeed);
                }
                // left bumper -Decrease Turn Speed
                else if (impGamepad1.a.isInitialPress()) {
                    telemetry.addData("decreasing speed by .01","");
                    config.driveSpeed -= .01;
                    if (config.driveSpeed < 0)
                        config.driveSpeed = 0;
                    telemetry.addData("speed is now ",config.driveSpeed);
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
