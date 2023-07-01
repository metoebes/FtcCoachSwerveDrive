package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;

public class Straffe extends AbstractTestCase {
    double angle;
    double magnitude;
    private final ImprovedGamepad impGamepad1;
    private final Telemetry telemetry;

    public Straffe(Robot _robot, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        this.impGamepad1 = _impGamepad1;
        this.telemetry = _telemetry;
        this.robot = _robot;
        testCaseName = "Straffe";
    }

    public boolean update() {
        switch (testCaseStep) {
            case 0: {
                startLogging(testCaseName);
                testCaseStep++;
                telemetry.addData("right joy stick", "straffe");
                telemetry.addData("L/R bumper", "turn");
                telemetry.addData("Press X", "to exit" + testCaseName);
                break;
            }
            case 1: {
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
                else {
                    // right stick active = drive toward heading
                    angle = impGamepad1.right_stick_angle;
                    magnitude = impGamepad1.right_stick_radius;
                    robot.updateHeading(angle, magnitude);
                }
                logData();
                break;
            }
            case 2: {
                stoplogging();
                testCaseStep++;
            }
            case 3: {
                break;
            }

        }
       return isComplete();
    }

    public boolean isComplete() {
        return (testCaseStep == 3);
    }

    public void logData() {
        if (datalog == null)
            return;

        try {
            datalog.addDataLine(
                    timer.time(),
                    testCaseName,
                    testCaseStep,
                    angle,
                    magnitude,
                    robot.forwardDrive_ChangeInHeading,
                    robot.reverseDrive_ChangeInHeading,
                    robot.centerMotorDirection,
                    robot.targetCenterMotorTicks,
                    robot.centerMotor.getCurrentPosition(),
                    robot.centerMotor.isBusy(),
                    robot.turnSpeed,
                    robot.driveSpeed
            );
        } catch (IOException e) {
            e.printStackTrace();

        }
    }
}