package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Diagnostics.StraffeDiagnostics;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;
import java.nio.charset.StandardCharsets;

public class Straffe extends AbstractTestCase {
    double angle;
    double magnitude;
    private final ImprovedGamepad impGamepad1;
    private final Telemetry telemetry;
    private boolean marker = false;
    private StraffeDiagnostics diagnostics;

    public Straffe(Robot _robot, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        this.impGamepad1 = _impGamepad1;
        this.telemetry = _telemetry;
        this.robot = _robot;
        testCaseName = "Straffe";
    }

    public void startLogging(String name) {
        super.startLogging(name);
        diagnostics = new StraffeDiagnostics(this.datalog);
    }

    public void help() {
        telemetry.addData("Straffe Test (current max speed = " + Float.toString(robot.config.driveSpeed) + ")", "");
        telemetry.addData("- right joy stick", "straffe");
        telemetry.addData("- L/R bumper", "turn");
        telemetry.addData("Press X", "to exit" + testCaseName);
    }
    public final int START_LOGGING = 0;
    public final int CHOOSE_TESTCASE = 1;
    public final int STOP_LOGGING = 2;
    public final int COMPLETED = 3;

    public boolean update() {
        switch (testCaseStep) {
            case START_LOGGING: {
                startLogging(testCaseName);
                help();
                testCaseStep++;
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }
            case CHOOSE_TESTCASE: {
                // right bumper -  CW pivot while pressed
                if (impGamepad1.right_bumper.isInitialPress()) {
                    robot.updateDirectionFacing(robot.getAngleFacing() + angle);
                }
                // right bumper - return to drive angle  when released
                else if (impGamepad1.right_bumper.isInitialRelease()) {
                    robot.updateHeading(robot.getAngleFacing(), 0);
                }
                // left bumper - CCW pivot while pressed
                else if (impGamepad1.left_bumper.isInitialPress()) {
                    robot.updateDirectionFacing(robot.getAngleFacing() - angle);
                }
                // left bumper - return to drive angle when released
                else if (impGamepad1.left_bumper.isInitialRelease()) {
                    robot.updateHeading(robot.getAngleFacing(), 0);
                }
                else if (impGamepad1.x.isInitialPress()) {
                    testCaseStep++;
                }
                // right joystick
                else {
                    // right stick active = drive toward heading
                    angle = impGamepad1.right_stick_angle;
                    magnitude = impGamepad1.right_stick_radius;
                    robot.updateHeading(angle, magnitude);
                }
                // Button A to add an annotation to the log data for debugging
                if (impGamepad1.a.isInitialPress()) {
                    marker =  true;
                }
                else {
                    marker = false;
                }
                logData();
                break;
            }
            case STOP_LOGGING: {
                stoplogging();
                testCaseStep++;
            }
            case COMPLETED: {
                break;
            }

        }
       return isComplete();
    }

    public boolean isComplete() {
        return (testCaseStep == 3);
    }

    public void logHeader() {
        diagnostics.logHeader();
    }

    public void logData() {

        if (diagnostics == null)
            return;

        diagnostics.update(
                timer.time(),
                testCaseStep,
                angle,
                magnitude,

                robot.forwardDrive_ChangeInHeading,
                robot.reverseDrive_ChangeInHeading,

                robot.centerMotorDirection,
                robot.turnSpeed,
                robot.rawTurnSpeed,

                robot.centerMotor.isBusy(),
                robot.centerMotor.getCurrentPosition(),
                robot.targetCenterMotorTicks,

                robot.driveDirection,
                robot.driveSpeed,
                robot.rawDriveSpeed,
                robot.frontRight.getCurrentPosition(),

                marker
        );
    }
}