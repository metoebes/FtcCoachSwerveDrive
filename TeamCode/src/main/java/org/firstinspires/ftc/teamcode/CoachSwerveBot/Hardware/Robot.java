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

    // Center turn motor
    // gobilda motor are CCW when given positive power
    public final int CCW = 1;
    public final int CW = -1;

    public final int FORWARD = 1;
    public final int BACKWARD = -1;

    // using a tetrix encoded 1140 counts per revolution
    public final double TICKS_PER_MOTOR_REV = 1440; // tetrix encoder
    public final double MIN_ANGLE_CHANGE = 360.0 / TICKS_PER_MOTOR_REV;

    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor centerMotor;
    public IMU imu;
    public Telemetry telemetry;
    public Config config;

    double robotRadiusInInches = 0;
    double wheelCircumferenceInInches = 0;
    public double changeInHeading;
    public double forwardDrive_ChangeInHeading;
    public double reverseDrive_ChangeInHeading;
    public double turnSpeed;
    public double driveSpeed;
    public double rawDriveSpeed;
    public double rawTurnSpeed;
    public int driveDirection = CCW;
    public double angleFacing;
    public double currentWheelAngle;
    public int targetCenterMotorTicks;
    public double targetDirectionAngle;
    public int centerMotorDirection= CCW;
    public boolean isUpdateDirectionFacing = false;
    private double startAngle;
    private int turnDirection;

    public void init(HardwareMap hardwareMap, Config _config, Telemetry _telemetry){

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        centerMotor = hardwareMap.dcMotor.get("center");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        centerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setPower(0);
        backLeft.setPower(0);
        centerMotor.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

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

    // The direction the robot is moving in. Not necessarily the direction it is facing
    public void updateHeading(double desiredAngle,double desiredDriveSpeed) {

        // turning at the moment...
        if (isUpdateDirectionFacing) {
            return;
        }
        // Drive speed is always positive. and must be <= 1.0
        if (desiredDriveSpeed >  1.0)
            desiredDriveSpeed = 1.0;

        // check if going in the opposite drive direction gets us there with less turning
        // or if the smallest angle change is to continue in present drive direction
        currentWheelAngle = toDegrees(centerMotor.getCurrentPosition());
        forwardDrive_ChangeInHeading =  AngleUtilities.closestAngle(currentWheelAngle, desiredAngle);        // ex: 0       180
        reverseDrive_ChangeInHeading = AngleUtilities.closestAngle(currentWheelAngle, desiredAngle + 180);   // ex: 180       0

        int previousDriveDirection = driveDirection;

        // smallest angle change is none or turning positive direction (CCW)
        if (Math.abs(forwardDrive_ChangeInHeading) <= Math.abs(reverseDrive_ChangeInHeading)) {
            changeInHeading = forwardDrive_ChangeInHeading;
            centerMotorDirection = CCW;
            driveDirection = FORWARD;
        }
        // smallest angle change is to change drive direction (CW)
        else {
            changeInHeading = reverseDrive_ChangeInHeading;
            centerMotorDirection = CW;
            driveDirection = BACKWARD;
        }

        // inflection point - changing direction..
        if (previousDriveDirection !=  driveDirection) {
            driveSpeed =  0;
        }

        // if change in direction is less than .24 degrees its not a significant change. (1 tick = .25 degree)
        targetDirectionAngle = currentWheelAngle;
        if (Math.abs(changeInHeading) <= MIN_ANGLE_CHANGE) {
            turnSpeed =0;
        }
        else {
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

        // scale the drive speed to within 0 to Max
        rawDriveSpeed = config.driveSpeed * (driveSpeed * driveDirection);
        rawTurnSpeed = config.turnSpeed * (turnSpeed * centerMotorDirection);

        centerMotor.setPower(rawTurnSpeed);
        backLeft.setPower(rawDriveSpeed);
        frontRight.setPower(rawDriveSpeed);
    }

    public void updateDirectionFacing( ) {
        if (centerMotor.isBusy()) {
            return; // wheels are not yet in turning position
        }
        //wheels are in turn position, start pivoting
        centerMotor.setPower(0);

        turnSpeed += config.turnSpeedIncrement;
        if (turnSpeed > config.turnSpeed) {
            turnSpeed = config.turnSpeed;
        }
        if (turnDirection == CCW)
            rawTurnSpeed = turnSpeed * config.turnSpeed * -1;
        else
            rawTurnSpeed = turnSpeed * config.turnSpeed * 1;

        frontRight.setPower(rawTurnSpeed);
        backLeft.setPower(rawTurnSpeed * -1);
    }

    public void beginFacingNewDirection(int direction) {
        updateHeading(45, 0);

        startAngle = getAngleFacing();
        turnDirection = direction;
        isUpdateDirectionFacing = true;

    }

    public void stopFacingNewDirection() {
        isUpdateDirectionFacing = false;
        double delta = getAngleFacing() - startAngle;
        if (turnDirection == CCW)
            updateHeading(delta, 0);
        else
            updateHeading(-delta, 0);
    }
}