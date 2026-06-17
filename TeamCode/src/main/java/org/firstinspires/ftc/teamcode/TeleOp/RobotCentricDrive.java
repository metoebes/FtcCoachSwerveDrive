package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Diagnostics.StraffeDiagnostics;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;

public class RobotCentricDrive {
    double angle;
    double magnitude;
    private boolean marker = false;

    public Robot robot = null;
    public ImprovedGamepad impGamepad1;
    public Telemetry telemetry;
    public Config config;
    public DataLogger datalog = null;
    private StraffeDiagnostics diagnostics;
    private String state = "alignWheels";
    public static double potError = 0;

    public RobotCentricDrive(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        this.impGamepad1 = _impGamepad1;
        this.telemetry = _telemetry;
        this.robot = _robot;
        this.config = _config;


        help();
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void help() {
        telemetry.clear();
        telemetry.addData("Robot Centric Drive (max speed = " + Float.toString(robot.config.driveSpeed) + ")", "");
        telemetry.addData("- right joy stick", "strafe speed & direction");
        telemetry.addData("- L/R bumper", "rotate to face");

    }

    public void start() {
        startLogging(this.getClass().getName());
    }

    public void update() {

        // on start, auto align
        if (state.equals("alignWheels")) {
            potError = robot.POT_FORWARD_VOLTAGE - robot.pot.getVoltage();
            if  (potError>robot.POT_ERROR_TOLERANCE) {
                robot.centerMotor.setPower(potError);
            } else {
                robot.centerMotor.setPower(0);
                state = "driving";
            }
            return;
        }

        // right bumper - CCW pivot while pressed
        if (impGamepad1.right_bumper.isInitialPress()) {
            robot.beginChangeHeading(robot.CCW);
        } else if (impGamepad1.right_bumper.isPressed()) {
            robot.updateHeading();
        } else if (impGamepad1.right_bumper.isInitialRelease()) {
            robot.endChangeHeading();
        }

        // left bumper - CCW pivot while pressed
        else if (impGamepad1.left_bumper.isInitialPress()) {
            robot.beginChangeHeading(robot.CW);
        } else if (impGamepad1.left_bumper.isPressed()) {
            robot.updateHeading();
        } else if (impGamepad1.left_bumper.isInitialRelease()) {
            robot.endChangeHeading();
        }

        // right stick active = drive toward heading
        angle = impGamepad1.right_stick_angle;
        // left stick active = speed
        magnitude = impGamepad1.right_stick_radius;
        if (impGamepad1.left_stick_y.getValue() < 0)
            angle += 180;

        // field centric - subtract whatever amount robot has already been turned
        angle -= Math.toDegrees(robot.pose.heading_radians);
        robot.updateDirection(angle, magnitude);

        // Button A to add an annotation to the log data for debugging
        if (impGamepad1.a.isInitialPress()) {
            marker = true;
        } else {
            marker = false;
        }
        logData();
    }

    public void stop()
    {
        stoplogging();
    }

    public void startLogging(String name) {
        if (config.useDataLogger) {
            try {
                datalog = new DataLogger(name + ".txt");
            } catch (IOException e) {
                e.printStackTrace();
            }
            diagnostics = new StraffeDiagnostics(this.datalog);
        }
    }

    public void stoplogging() {
        if (datalog!=null) {
            datalog.close();
            datalog = null;
        }
    }

    public void logData() {

        if (diagnostics == null)
            return;

        diagnostics.update(
                state,

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
