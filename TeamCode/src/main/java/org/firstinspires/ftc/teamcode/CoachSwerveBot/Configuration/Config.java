package org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

public class Config {
    public static final float DEFAULT_SPEED_UP = (float).005;
    public static int TANK_TURN = 1;

    public int turnStyle = TANK_TURN;
    public float maxTurnSpeed = (float) 0.5;
    public boolean encoders = false;
    public float driveSpeed = (float) .5;
    public float driveSpeedIncrement = DEFAULT_SPEED_UP;
    public float turnSpeed = (float)  .5;
    public float turnSpeedIncrement = 2 * DEFAULT_SPEED_UP;

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
