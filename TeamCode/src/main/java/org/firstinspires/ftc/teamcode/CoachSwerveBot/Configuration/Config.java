package org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;

public class Config {
    public static final float MAX_DRIVE_SPEED = (float)1.0;
    public static final float DEFAULT_SPEED_UP = (float).005;

    public float maxTurnSpeed = (float) 0.5;
    public boolean encoders = false;
    public float driveSpeed = (float) .5;
    public float turnSpeed = (float)1.00;
    public float driveSpeedIncrement = DEFAULT_SPEED_UP;

    public ImprovedGamepad impGamepad1;
    public Robot robot = null;

    public void init(Robot robot,ImprovedGamepad impGamepad1) {
           this.robot = robot;
           this.impGamepad1 = impGamepad1;
    }

    public void update() {
        // A button decreases drive Speed
        if (impGamepad1.a.isInitialPress()) {
            driveSpeedIncrement -= .001;
        }
        // Y button increases drive Speed
        if (impGamepad1.y.isInitialPress()) {
            driveSpeedIncrement += .001;
        }
        if (driveSpeedIncrement <= DEFAULT_SPEED_UP) {
            driveSpeedIncrement = DEFAULT_SPEED_UP;
        }
        // enable/disable the use of encoders
        if (impGamepad1.b.isInitialPress()) {
            encoders = !encoders;
        }

        if (encoders) {
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Start button resets the IMU
        if (impGamepad1.start.isInitialPress()) {
            robot.centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Left Bumper increases turn speed
        if (impGamepad1.left_bumper.isPressed()) turnSpeed += maxTurnSpeed;
            // Right bumper decreases turn speed
        else if (impGamepad1.right_bumper.isPressed()) turnSpeed -= maxTurnSpeed;

    }


}
