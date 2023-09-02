package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;

public class Encoder extends AbstractTestCase {
    private final int TEST_START_LOGGING = 0;
    private final int TEST_HAND_TURN_WHEEL = 1;
    private final int TEST_STOP_LOGGING = 2;
    private final int TEST_COMPLETE = 3;
    private final int TEST_BL_ENCODER = 4;
    private final int TEST_FR_ENCODER = 5;
    private final int TEST_BL_FR_ENCODER = 6;
    private final int REVOLUTIONS = 10;

    private int target  = 0;

    public Encoder(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        super("Spin DriveMotor", _robot, _config, _impGamepad1, _telemetry);
    }

    public void  help() {
        telemetry.addData("Encoder Test", "");
        telemetry.addData("- Rotate a wheel and observer encoder counts", "");
        telemetry.addData("- Press A to go "+REVOLUTIONS+" revolution on backLeft motor", "");
        telemetry.addData("- Press Y to go "+REVOLUTIONS+" revolution on rightFront motor", "");
        telemetry.addData("- Press B to go "+REVOLUTIONS+" revolution on both motor", "");
        telemetry.addData("Press X", "to exit" + testCaseName);
        if ( robot.frontRight instanceof DcMotorEx) {
            telemetry.addLine("Yes, instance of DCMotorEx");
            telemetry.addLine("PIDF coefficients ");
            PIDFCoefficients pidf = ((DcMotorEx) robot.frontRight).getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("algorithm", pidf.algorithm);
            telemetry.addData("p", pidf.p);
            telemetry.addData("i", pidf.i);
            telemetry.addData("d", pidf.d);
            telemetry.addData("f", pidf.f);
        }
        else {
            telemetry.addLine("No, is *not* instance of DCMotorEx");
        }
    }

    public void telemetryDetails() {
        telemetry.addLine();
        telemetry.addData("Back Left", "");
        telemetry.addData("- target", robot.backLeft.getTargetPosition());
        telemetry.addData("- Count", robot.backLeft.getCurrentPosition());
        telemetry.addData("- speed", robot.backLeft.getPower());
        telemetry.addLine();
        telemetry.addData("Front Right", "");
        telemetry.addData("- target", robot.frontRight.getTargetPosition());
        telemetry.addData("- Count", robot.frontRight.getCurrentPosition());
        telemetry.addData("- speed", robot.frontRight.getPower());
    }
    public boolean update() {
        switch (testCaseStep) {
            case TEST_START_LOGGING: {
                startLogging(testCaseName);
                testCaseStep++;
                break;
            }
            case TEST_HAND_TURN_WHEEL: {

                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                help();
                telemetry.addData("BL Count", robot.backLeft.getCurrentPosition());
                telemetry.addData("FR Count", robot.frontRight.getCurrentPosition());

                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep = TEST_STOP_LOGGING;

                } else if (impGamepad1.a.isInitialPress()) {
                    target= (int)robot.TICKS_PER_MOTOR_REV * REVOLUTIONS;
                    robot.backLeft.setTargetPosition(target);
                    robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backLeft.setPower(robot.config.driveSpeed * robot.FORWARD);
                    testCaseStep = TEST_BL_ENCODER;

                } else if (impGamepad1.y.isInitialPress()) {
                    target= (int)robot.TICKS_PER_MOTOR_REV * REVOLUTIONS;
                    robot.frontRight.setTargetPosition(target);
                    robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.frontRight.setPower(robot.config.driveSpeed * robot.FORWARD);
                    testCaseStep = TEST_FR_ENCODER;

                } else if (impGamepad1.b.isInitialPress()) {
                    target= (int)robot.TICKS_PER_MOTOR_REV * REVOLUTIONS;

                    robot.frontRight.setTargetPosition(target);
                    robot.backLeft.setTargetPosition(target);

                    robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.frontRight.setPower(robot.config.driveSpeed * robot.FORWARD);
                    robot.backLeft.setPower(robot.config.driveSpeed * robot.FORWARD);

                    testCaseStep = TEST_FR_ENCODER;
            }
                logData();
                break;
            }
            case TEST_STOP_LOGGING: {
                stoplogging();
                testCaseStep++;
                break;
            }
            case TEST_COMPLETE:
                break;

            case TEST_BL_ENCODER: {
                if (impGamepad1.x.isInitialPress()) {
                    robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.backLeft.setPower(0);
                    testCaseStep = TEST_HAND_TURN_WHEEL;
                }
                telemetry.addData("Rotate BackLeft", "");
                telemetry.addData("- Press X", "to exit" + testCaseName);
                telemetryDetails();
                break;
            }
            case TEST_FR_ENCODER: {
                if (impGamepad1.x.isInitialPress()) {
                    robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.frontRight.setPower(0);
                    testCaseStep = TEST_HAND_TURN_WHEEL;
                }
                telemetry.addData("Rotate FrontRight", "");
                telemetry.addData("- Press X", "to exit" + testCaseName);
                telemetryDetails();
                break;
            }
            case TEST_BL_FR_ENCODER: {
                if (impGamepad1.x.isInitialPress()) {
                    robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.frontRight.setPower(0);
                    testCaseStep = TEST_HAND_TURN_WHEEL;
                }
                telemetry.addData("Rotate FrontRight", "");
                telemetry.addData("- Press X", "to exit" + testCaseName);
                telemetryDetails();
                break;
            }

        }
        return isComplete();
    }
    public boolean isComplete() {
        return (testCaseStep == TEST_COMPLETE);
    }
    public void logData(){
        if (datalog == null)
            return;

        try {
            datalog.addDataLine(
                    timer.time(),
                    testCaseName,
                    testCaseStep,
                    target,
                    robot.centerMotorDirection,
                    robot.targetCenterMotorTicks,
                    robot.centerMotor.getCurrentPosition(),
                    robot.centerMotor.isBusy(),
                    robot.turnSpeed
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
