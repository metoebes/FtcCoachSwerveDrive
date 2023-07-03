package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;

public class DriveMotor extends AbstractTestCase {
    private final int TEST_START_LOGGING = 0;
    private final int WAIT_FOR_CHOICE = 1;
    private final int TEST_STOP_LOGGING = 2;
    private final int TEST_COMPLETE = 3;
    private final int TEST_SPIN_FR_FWD = 4;
    private final int TEST_SPIN_BL_FWD = 5;
    private final int TEST_SPIN_FR_BCK = 6;
    private final int TEST_SPIN_BL_BCK = 7;

    private final Telemetry telemetry;
    private final ImprovedGamepad impGamepad1;
    public ElapsedTime timer = new ElapsedTime();
    // private int target  = 0;
    private int duration = 5;

    public DriveMotor(Robot _robot, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        this.testCaseName = "DriveMotor";
        this.robot = _robot;
        telemetry = _telemetry;
        impGamepad1 = _impGamepad1;
    }

    public void  help() {
        telemetry.addData("Drive Motor Test", "");
        telemetry.addData("- Press A spin BL 1sec F/B", "");
        telemetry.addData("- Press Y spin FR 1sec F/B", "");
        telemetry.addData("Press X", "to exit" + testCaseName);
    }

    public void telemetryDetails() {
        telemetry.addLine();
        telemetry.addLine("Back Left");
        telemetry.addData("BL Power", robot.backLeft.getPower());
        telemetry.addData("BL Count", robot.backLeft.getCurrentPosition());
        telemetry.addLine("Front Right");
        telemetry.addData("FR Power", robot.frontRight.getPower());
        telemetry.addData("FR Count", robot.frontRight.getCurrentPosition());
    }
    public boolean update() {
        switch (testCaseStep) {
            case TEST_START_LOGGING: {
                startLogging(testCaseName);
                logHeader();
                testCaseStep++;
                break;
            }
            case WAIT_FOR_CHOICE: {
                robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.backLeft.setPower(0);
                robot.frontRight.setPower(0);

                help();
                telemetryDetails();

                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep = TEST_STOP_LOGGING;

                } else if (impGamepad1.y.isInitialPress()) {
                    timer.reset();
                    testCaseStep = TEST_SPIN_FR_FWD;

                } else if (impGamepad1.a.isInitialPress()) {
                    timer.reset();
                    testCaseStep = TEST_SPIN_BL_FWD;
                }
                logData();
                break;
            }
            case TEST_STOP_LOGGING: {
                stoplogging();

                robot.backLeft.setPower(0);
                robot.frontRight.setPower(0);

                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                testCaseStep++;
                break;
            }
            case TEST_COMPLETE:
                break;

            case TEST_SPIN_BL_FWD: {
                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep = WAIT_FOR_CHOICE;
                }
                int remaining = (int)((duration - timer.seconds())* 1000);

                telemetry.addData("Spin BackLeft 1sec Forward", "");
                telemetry.addData("- Press X", "to exit" + testCaseName);
                telemetry.addData("remaining msec", remaining);
                telemetryDetails();

                robot.backLeft.setPower(robot.config.driveSpeed * robot.FORWARD);
                if (remaining <= 0) {
                    timer.reset();
                    testCaseStep = TEST_SPIN_BL_BCK;
                }
                break;
            }
            case TEST_SPIN_BL_BCK: {
                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep = WAIT_FOR_CHOICE;
                }
                int remaining = (int)((duration - timer.seconds())* 1000);
                telemetry.addData("Spin BackLeft 1sec Forward", "");
                telemetry.addData("- Press X", "to exit" + testCaseName);
                telemetry.addData("remaining msec", remaining);
                telemetryDetails();

                robot.backLeft.setPower(robot.config.driveSpeed * robot.BACKWARD);
                if (remaining <= 0) {
                    testCaseStep = WAIT_FOR_CHOICE;
                }
                break;
            }

            case TEST_SPIN_FR_FWD: {
                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep = WAIT_FOR_CHOICE;
                }
                int remaining = (int)((duration - timer.seconds())* 1000);
                telemetry.addData("Spin FrontRight 1sec FWD", "");
                telemetry.addData("- Press X", "to exit" + testCaseName);
                telemetry.addData("remaining msec", remaining);
                telemetryDetails();

                robot.frontRight.setPower(robot.config.driveSpeed * robot.FORWARD);
                if (remaining <= 0) {
                    timer.reset();
                    testCaseStep = TEST_SPIN_FR_BCK;
                }
                break;
            }
            case TEST_SPIN_FR_BCK: {
                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep = WAIT_FOR_CHOICE;
                }
                int remaining = (int)((duration - timer.seconds())* 1000);
                telemetry.addData("Spin FrontRight 1sec BCK", "");
                telemetry.addData("- Press X", "to exit" + testCaseName);
                telemetry.addData("remaining msec", remaining);
                telemetryDetails();

                robot.frontRight.setPower(robot.config.driveSpeed * robot.BACKWARD);
                if (remaining <= 0) {
                    testCaseStep = WAIT_FOR_CHOICE;
                }
                break;
            }

        }
        return isComplete();
    }
    public boolean isComplete() {
        return (testCaseStep == TEST_COMPLETE);
    }

    public void logHeader() {
        if (datalog == null)
            return;

        try {
            datalog.addDataLine(
                    "time",
                    "testCaseName",
                    "testCaseStep",
                    "remaining",
                    "robot.backLeft.getPower()",
                    "robot.backLeft.getCurrentPosition()",
                    "robot.frontRight.getPower()",
                    "robot.frontRight.getCurrentPosition()"
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void logData(){
        if (datalog == null)
            return;

        int remaining = 0;
        if (testCaseStep > TEST_COMPLETE )
            remaining = (int)((duration - timer.seconds())* 1000);

        try {
            datalog.addDataLine(
                    timer.time(),
                    testCaseName,
                    testCaseStep,
                    remaining,
                    robot.backLeft.getPower(),
                    robot.backLeft.getCurrentPosition(),
                    robot.frontRight.getPower(),
                    robot.frontRight.getCurrentPosition()
                    );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
