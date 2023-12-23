package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Diagnostics.StraffeDiagnostics;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

public class RobotCentricDrive extends AbstractTestCase {
    double angle;
    double magnitude;
    private boolean marker = false;
    private StraffeDiagnostics diagnostics;

    public RobotCentricDrive(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        super("Robot Centric Drive", _robot, _config, _impGamepad1, _telemetry);
    }

    public void startLogging(String name) {
        super.startLogging(name);
        diagnostics = new StraffeDiagnostics(this.datalog);
    }

    public void help() {
        telemetry.clear();
        // telemetry.addData("Robot Centric Drive (max speed = " + Float.toString(robot.config.driveSpeed) + ")", "");
        telemetry.addData("right joy stick", "straffe");
        // telemetry.addData("- L/R bumper", "rotate to face");
        telemetry.addLine("Press X to return to main menu");

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
                timer.reset();
                testCaseStep++;
                robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            }
            case CHOOSE_TESTCASE: {
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

                // left bumper - CCW pivot while pressed
                else if (impGamepad1.left_bumper.isInitialPress()) {
                    robot.beginChangeHeading(robot.CW);
                }
                else if (impGamepad1.left_bumper.isPressed()) {
                    robot.updateHeading();
                }
                else if (impGamepad1.left_bumper.isInitialRelease()) {
                    robot.endChangeHeading();
                }

                else if (impGamepad1.x.isInitialPress()) {
                    testCaseStep++;
                }

                // right stick active = drive toward heading
                angle = impGamepad1.right_stick_angle;
                // left stick active = speed
                magnitude = impGamepad1.right_stick_radius;
                if (impGamepad1.left_stick_y.getValue() < 0 )
                    angle +=180;

                // field centric - subtract whatever amount robot has already been turned
                angle -= Math.toDegrees(robot.pose.heading_radians);
                robot.updateDirection(angle, magnitude);
                
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

                robot.centerMotorDirection,
                robot.turnSpeed,
                0,

                robot.centerMotor.isBusy(),
                robot.centerMotor.getCurrentPosition(),
                robot.targetCenterMotorTicks,

                robot.driveDirection,
                robot.driveSpeed,
                0,
                robot.frontRight.getCurrentPosition(),

                marker
        );
    }
}