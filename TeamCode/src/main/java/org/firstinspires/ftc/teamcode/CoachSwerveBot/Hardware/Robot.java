package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

public class Robot implements IRobot {

    // gobilda motor are CCW when given positive power
    public final int CCW = 1;
    public final int CW = -1;
    // using a tetrix encoded 1140 counts per revolution
    public final double TICKS_PER_MOTOR_REV = 1440; // tetrix encoder
    public final double MIN_ANGLE_CHANGE = 360.0 / TICKS_PER_MOTOR_REV;

    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor centerMotor;
    public IMU imu;
    public Telemetry telemetry;
    Config config;

    double robotRadiusInInches = 0;
    double wheelCircumferenceInInches = 0;
    public double changeInHeading;
    public double forwardDrive_ChangeInHeading;
    public double reverseDrive_ChangeInHeading;
    public double turnSpeed;
    public double driveSpeed;
    public int driveDirection = CCW;
    public double angleFacing;
    public double currentWheelAngle;
    public int targetCenterMotorTicks;
    public double targetDirectionAngle;
    public int centerMotorDirection= CCW;
    public boolean isUpdateDirectionFacing = false;

    public void init(HardwareMap hardwareMap, Config _config, Telemetry _telemetry){

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        centerMotor = hardwareMap.dcMotor.get("center");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontRight.setPower(0);
        backLeft.setPower(0);
        centerMotor.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = _telemetry;
        config = _config;
    }

    double toDegrees(int ticks) {
        return ticks / (TICKS_PER_MOTOR_REV ) * 360.0;
    }
    int toTicks( double angle) {
        return (int) (angle /360 *(TICKS_PER_MOTOR_REV ));
    }

    public double getAngleFacing() {
        YawPitchRollAngles orientation = this.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void updateDirectionFacing(double faceTowardDesiredAngle) {
        isUpdateDirectionFacing = true;
        // turn wheels to 45 degree angle
        updateHeading(45, 0);
        double centerMotorAngle = toDegrees(centerMotor.getCurrentPosition());
        if (Math.abs(centerMotorAngle) - 45 >= MIN_ANGLE_CHANGE) {
            return; // wheels are not yet in turning position
        }
        
        // pivot
        angleFacing = getAngleFacing();
        forwardDrive_ChangeInHeading = AngleUtilities.closestAngle(angleFacing, faceTowardDesiredAngle);
        reverseDrive_ChangeInHeading = AngleUtilities.closestAngle(angleFacing, faceTowardDesiredAngle + 180);

        if (Math.abs(forwardDrive_ChangeInHeading) <= Math.abs(reverseDrive_ChangeInHeading)) {
            changeInHeading = forwardDrive_ChangeInHeading;
            driveDirection = CCW;
        }
        // smallest angle change is to change drive direction
        else {
            changeInHeading = reverseDrive_ChangeInHeading;
            driveDirection = CW;
        }
        // determine Target ticks (for trapezoidal speed reduction)
        driveSpeed = config.driveSpeed;
        if (Math.abs(changeInHeading) < 5) {
            driveSpeed *= Math.abs(changeInHeading) / 5;
        }

        frontRight.setPower(driveSpeed * driveDirection);
        backLeft.setPower(driveSpeed * driveDirection * -1);
    }

    // The direction the robot is moving in. Not necessarily the direction it is facing
    public void updateHeading(double desiredAngle,double desiredDriveSpeed) {

        if (isUpdateDirectionFacing) {
            if (isPivoting())
                return; // turning at the moment..
            isUpdateDirectionFacing = false;
        }

        currentWheelAngle = toDegrees(centerMotor.getCurrentPosition());
        forwardDrive_ChangeInHeading =  AngleUtilities.closestAngle(currentWheelAngle, desiredAngle);
        reverseDrive_ChangeInHeading = AngleUtilities.closestAngle(currentWheelAngle, desiredAngle + 180);
        
        // check if going in the opposite drive direction gets us there with less turning
        // smallest angle change is to continue in present drive direction
        if (Math.abs(forwardDrive_ChangeInHeading) <= Math.abs(reverseDrive_ChangeInHeading)) {
            changeInHeading = forwardDrive_ChangeInHeading;
        }
        // smallest angle change is to change drive direction
        else {
            changeInHeading = reverseDrive_ChangeInHeading;
            driveDirection *= -1;
            driveSpeed = 0;
        }
        // turn in opposite direction
        if (Math.signum(changeInHeading) != centerMotorDirection) {
            centerMotorDirection = -1 * centerMotorDirection;
        }
        // if less than .24 degrees its not a significant change. (1 tick = .25 degree)
        targetDirectionAngle = currentWheelAngle;        
        if (Math.abs(changeInHeading) > MIN_ANGLE_CHANGE) {
            targetDirectionAngle += changeInHeading;
            targetCenterMotorTicks = toTicks(targetDirectionAngle);

            // trapezoidal slow down as approaching target angle.
            // ramp down speed when withing 5 degrees of target.
            turnSpeed = config.turnSpeed;
            if (Math.abs(changeInHeading) < 5) {
                turnSpeed *= Math.abs(changeInHeading) / 5;
            }

            centerMotor.setTargetPosition(targetCenterMotorTicks);
            centerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            turnSpeed *= centerMotorDirection;
            centerMotor.setPower(turnSpeed);
        }
        // Set the drive speed
        if  (desiredDriveSpeed ==0) {
            driveSpeed = 0;
        }
        else {
            driveSpeed += config.driveSpeedIncrement;
            if (driveSpeed > desiredDriveSpeed) {
                driveSpeed = desiredDriveSpeed;
            }
        }
        backLeft.setPower(driveSpeed * driveDirection);
        frontRight.setPower(driveSpeed * driveDirection);
    }

    public boolean isPivoting() {
        return centerMotor.isBusy() || backLeft.isBusy() || frontRight.isBusy();
    }
}