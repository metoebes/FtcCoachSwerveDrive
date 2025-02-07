package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases.AbstractTestCase;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;


public class ConfigTurnSpeed extends AbstractTestCase {

    public ConfigTurnSpeed(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        super("Set Turn Speed", _robot, _config, _impGamepad1, _telemetry);
    }

    private void help() {
        telemetry.addData("L/R bumper", "-/+ turn speed");
        telemetry.addData("   Turn Speed", config.turnSpeed);
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
                    config.turnSpeed += .1;
                    if (config.turnSpeed > 1)
                        config.turnSpeed = 1;
                }
                // left bumper -Decrease Turn Speed
                else if (impGamepad1.a.isInitialPress()) {
                    config.turnSpeed -= .1;
                    if (config.turnSpeed < 0)
                        config.turnSpeed = 0;
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
