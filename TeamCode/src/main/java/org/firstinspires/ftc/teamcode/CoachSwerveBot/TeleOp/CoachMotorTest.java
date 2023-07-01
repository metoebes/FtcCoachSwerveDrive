package org.firstinspires.ftc.teamcode.CoachSwerveBot.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "Coach Motor Test", group = "coach")
public class CoachMotorTest extends OpMode{

    final float DEAD_ZONE = (float) 0.3;
    final float TURN_SPEED = (float) 0.5;
    float center = 0;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor centerMotor;

    @Override
    public void init(){

        centerMotor = hardwareMap.dcMotor.get("center");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontRight.setPower(0);
        backLeft.setPower(0);
        centerMotor.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        help();
    }

    public void help() {
        telemetry.addData("Left Joystick = ","frontRight Motor");
        telemetry.addData("Right Joystick = ","backLeft Motor");
        telemetry.addData("Left/right bumper = ","center Motor");
    }

    public void processDriveActions() {
        float  left = (Math.abs(gamepad1.left_stick_y) < DEAD_ZONE) ? 0: gamepad1.left_stick_y;
        float  right= (Math.abs(gamepad1.right_stick_y) < DEAD_ZONE) ? 0: gamepad1.right_stick_y;

        if (gamepad1.left_bumper) center = TURN_SPEED;
        else if (gamepad1.right_bumper) center -= TURN_SPEED;
        else center = 0;

        backLeft.setPower(left);
        frontRight.setPower(right);
        centerMotor.setPower(center);

        telemetry.addData("Center Speed", center);
        telemetry.addData("Front Right Speed", right);
        telemetry.addData("Back Left Speed", left);
        telemetry.addData("------", "-----");
        telemetry.addData("Center Position", centerMotor.getCurrentPosition());
        telemetry.addData("Front Right Position", frontRight.getCurrentPosition());
        telemetry.addData("Back Left Position", backLeft.getCurrentPosition());

    }


    @Override
    public void loop() {
        processDriveActions();
        telemetry.update();
    }

}
