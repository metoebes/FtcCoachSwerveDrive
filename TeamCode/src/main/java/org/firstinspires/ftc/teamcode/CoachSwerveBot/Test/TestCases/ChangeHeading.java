package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;

public class ChangeHeading extends AbstractTestCase {
    int ticks = 0;
    double targetAngleDegrees = 0;

    public final int START_LOGGING = 0;
    public final int EXECUTE_TESTCASE = 1;
    public final int STOP_LOGGING = 2;
    public final int COMPLETED = 3;
    public final int TURNTONEWHEADING = 4;

    public ChangeHeading(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        super("Rotate Heading", _robot, _config, _impGamepad1, _telemetry);
    }

    public void help() {
        telemetry.clear();
        telemetry.addLine("Robot Heading");
        telemetry.addLine("- R/L bumper: rotate CCW/CW ");
        telemetry.addLine("Press X to return to main menu");
    }

    public boolean update() {
        switch (testCaseStep) {
            case START_LOGGING: {
                startLogging(testCaseName);
                help();
                timer.reset();
                ticks = robot.frontRight.getCurrentPosition();
                testCaseStep++;
            }
            case EXECUTE_TESTCASE: {
                // right bumper - CCW pivot while pressed
                if (impGamepad1.right_bumper.isInitialPress()) {
                    robot.beginChangeHeading(robot.CCW);
                }
                else if (impGamepad1.right_bumper.isPressed()) {
                    robot.updateHeading();
                }
                else if (impGamepad1.right_bumper.isInitialRelease()) {
                    robot.endChangeHeading();
                }

                // left bumper - CW pivot while pressed
                else if (impGamepad1.left_bumper.isInitialPress()) {
                    robot.beginChangeHeading(robot.CW);
                }
                else if (impGamepad1.left_bumper.isPressed()) {
                    robot.updateHeading();
                }
                else if (impGamepad1.left_bumper.isInitialRelease()) {
                    robot.endChangeHeading();
                }

                if (impGamepad1.y.isInitialPress()) {
                    robot.beginChangeHeading(robot.CW);
                    targetAngleDegrees = robot.getImuHeadingInDegrees() + 90;
                    testCaseStep = TURNTONEWHEADING;
                }
                // Press X to end test
                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep++;
                }
                logData();
                break;
            }
            case STOP_LOGGING: {
                stoplogging();
                testCaseStep++;
                break;
            }
            case COMPLETED: {
                if (robot.isTurningToNewHeading()) {
                    robot.endChangeHeading();
                }
                break;
            }

            case TURNTONEWHEADING: {
                if (robot.getImuHeadingInDegrees() < targetAngleDegrees) {
                    robot.updateHeading();
                }
                else {
                    robot.endChangeHeading();
                    targetAngleDegrees = 0;
                    testCaseStep = EXECUTE_TESTCASE;
                }
                // Press X to end test
                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep=STOP_LOGGING;
                }
            }
        }
        return isComplete();
    }
    public boolean isComplete() {
        return (testCaseStep == 3);
    }
    public void logData(){
        if (datalog == null)
            return;

        try {
            datalog.addDataLine(
                    timer.time(),
                    testCaseName,
                    testCaseStep,
                    robot.getImuHeadingInDegrees(),
                    robot.robotPose.heading_radians,
                    ticks
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
