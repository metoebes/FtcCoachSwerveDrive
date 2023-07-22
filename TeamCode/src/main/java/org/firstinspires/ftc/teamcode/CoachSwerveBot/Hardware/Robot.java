package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public final double WHEEL_RADIUS_INCHES = 1.5;
    public final double INCHES_PER_REV = 2 * Math.PI * WHEEL_RADIUS_INCHES ;
    public final double DIAG_DST_BETWEEN_WHEELS_INCHES = 13.0;
    public final double Y_DST_BETWEEN_WHEELS_INCHES = 9.2;
    public final double X_DST_BETWEEN_WHEELS_INCHES = 9.2;

    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor centerMotor;
    public IMU imu;
    public Telemetry telemetry;
    public Config config;

    public double turnSpeed;
    public double driveSpeed;
    public double rawDriveSpeed;
    public double rawTurnSpeed;
    public int driveDirection = CCW;
    public double currentWheelAngle_degrees;
    public int targetCenterMotorTicks;
    public double targetDirectionAngle;
    public int centerMotorDirection= CCW;



    private class RotateRobotInfo {
        public boolean isUpdateDirectionFacing = false;
        public double originalAngle;
        public int turnDirection;
    };
    RotateRobotInfo rotateRobotInfo = new RotateRobotInfo();

    private class DriveWheelInfo {
        private double lastKnown_x_inches =0;
        private double lastKnown_y_inches =0;
        private int lastKnown_ticks = 0;

        public DriveWheelInfo(double x, double y) {
            lastKnown_x_inches = x;
            lastKnown_y_inches = y;
        }
    };
    DriveWheelInfo backLeftInfo = new DriveWheelInfo( 0.0, 0.0);
    DriveWheelInfo frontRightInfo = new DriveWheelInfo(X_DST_BETWEEN_WHEELS_INCHES,  - Y_DST_BETWEEN_WHEELS_INCHES);

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

        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = _telemetry;
        config = _config;

        drawField();
    }
    public void drawField() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        drawAxis(canvas);
        drawRobot(canvas);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }
    public void drawAxis(Canvas canvas) {
            canvas
                .setStroke("magenta")
                .setStrokeWidth(1)
                .strokeLine(0, 0, 6*12, 0)

                .strokeLine(5.5*12, 3, 6*12, 0)
                .strokeLine( 5.5*12, -3, 6*12, 0)

                .strokeLine( 5.25*12,  12, 5.75*12,  6)
                .strokeLine( 5.25*12,   6, 5.75*12, 12)

                .setStrokeWidth(1)
                .strokeLine(0, 0, 0, 6*12)

                .strokeLine(-3, 5.5*12, 0, 6*12)
                .strokeLine( 3, 5.5*12, 0, 6*12)

                .strokeLine(-12, 4.5*12, -6, 5.5*12)
                .strokeLine( -12, 5.5*12, -9, 5.0*12)
        ;
        FtcDashboard dashboard = FtcDashboard.getInstance();
    }

    public void drawWheel(Canvas canvas, DriveWheelInfo wheelInfo) {
        // Create a straight line (4" long) with arrow at the end
        double x[] = { -WHEEL_RADIUS_INCHES, WHEEL_RADIUS_INCHES, WHEEL_RADIUS_INCHES-1,  WHEEL_RADIUS_INCHES-1};
        double y[] = { 0, 0, 1, -1};
        double currentWheelAngle_radians = currentWheelAngle_degrees *  Math.PI / 180.0;
        double cos = Math.cos(currentWheelAngle_radians);
        double sin = Math.sin(currentWheelAngle_radians);

        telemetry.addData("angle in degrees", AngleUtilities.getPositiveNormalizedAngle(currentWheelAngle_degrees));
        telemetry.addData("cos", cos);
        telemetry.addData("sin", sin);

        // Rotate line to point in the direction the robot is heading
        for (int ii=0; ii<x.length; ii++) {
            double nx = x[ii] * cos - y[ii] * sin;
            double ny = y[ii] * cos + x[ii] * sin;
            x[ii] = nx;
            y[ii] = ny;
        }
        // translate the line to the location the robot BL wheel is currently at
        for (int ii=0; ii<x.length; ii++) {
            x[ii] += wheelInfo.lastKnown_x_inches;
            y[ii] += wheelInfo.lastKnown_y_inches;
        }
        canvas.setStroke("black");
        canvas.setStrokeWidth(0);
        canvas.strokeLine(x[0], y[0], x[1], y[1]);
        canvas.strokeLine(x[1], y[1], x[2], y[2]);
        canvas.strokeLine(x[1], y[1], x[3], y[3]);
    }

    public void drawRobot(Canvas canvas) {
        canvas.setStroke("gray");
        double x = (backLeftInfo.lastKnown_x_inches + frontRightInfo.lastKnown_x_inches)/2.0;
        double y = (backLeftInfo.lastKnown_y_inches + frontRightInfo.lastKnown_y_inches)/2.0;
        canvas.strokeCircle( x, y, DIAG_DST_BETWEEN_WHEELS_INCHES/2.0 );

        drawWheel(canvas, this.backLeftInfo);
        drawWheel(canvas, this.frontRightInfo);
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

    public void updateWheelLocation(DriveWheelInfo wheelInfo, int current_ticks) {

        int traveled_ticks = current_ticks - wheelInfo.lastKnown_ticks;
        double traveled_inches = traveled_ticks / TICKS_PER_MOTOR_REV * INCHES_PER_REV;

        double angle_radians = 0;
        double delta_x = 0;
        double delta_y = 0;

        // going straight
        if (!rotateRobotInfo.isUpdateDirectionFacing) {
            //angle_radians = currentWheelAngle_degrees *  Math.PI / 180.0;
            //delta_x = traveled_inches * Math.cos(angle_radians);
            //delta_y = traveled_inches * Math.sin(angle_radians);
        }
        // turning (both wheels at 45 degree angle pivoting around center of robot)
        else {
            angle_radians = traveled_ticks / TICKS_PER_MOTOR_REV;
            delta_x = DIAG_DST_BETWEEN_WHEELS_INCHES/2.0 - (traveled_inches * Math.cos(angle_radians));
            delta_y = DIAG_DST_BETWEEN_WHEELS_INCHES/2.0 - (traveled_inches * Math.sin(angle_radians));
            telemetry.addData("angle(radians)", angle_radians);
            telemetry.addData("delta_x",delta_x);
            telemetry.addData("delta_y", delta_y);
        }

        wheelInfo.lastKnown_ticks = current_ticks;
        wheelInfo.lastKnown_x_inches += delta_x;
        wheelInfo.lastKnown_y_inches += delta_y;

    }
    public void updateFieldLocation() {
        int current_BL_ticks = backLeft.getCurrentPosition();
        int current_FR_ticks = frontRight.getCurrentPosition();
        currentWheelAngle_degrees = toDegrees(centerMotor.getCurrentPosition());

        updateWheelLocation(this.backLeftInfo, current_BL_ticks);
        updateWheelLocation(this.frontRightInfo, current_FR_ticks);

        drawField();
    }
    // The direction the robot is moving in. Not necessarily the direction it is facing
    public void updateHeading(double desiredAngle,double desiredDriveSpeed) {
        // update our understanding of were we are on the field.
        updateFieldLocation();

        // turning at the moment...
        if (rotateRobotInfo.isUpdateDirectionFacing) {
            if (this.config.turnStyle != config.TANK_TURN) {
                return;
            }
        }
        // Drive speed is always positive. and must be <= 1.0
        if (desiredDriveSpeed >  1.0)
            desiredDriveSpeed = 1.0;

        // check if going in the opposite drive direction gets us there with less turning
        // or if the smallest angle change is to continue in present drive direction
        double forwardDrive_ChangeInHeading =  AngleUtilities.closestAngle(currentWheelAngle_degrees, desiredAngle);        // ex: 0       180
        double reverseDrive_ChangeInHeading = AngleUtilities.closestAngle(currentWheelAngle_degrees, desiredAngle + 180);   // ex: 180       0

        int previousDriveDirection = driveDirection;
        double changeInHeading;

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
        targetDirectionAngle = currentWheelAngle_degrees;
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

        if (rotateRobotInfo.isUpdateDirectionFacing) {
            if (this.config.turnStyle == Config.TANK_TURN) {
                if (rotateRobotInfo.turnDirection == CCW) {
                    frontRight.setPower(rawDriveSpeed);
                    backLeft.setPower(-rawDriveSpeed);
                } else {
                    frontRight.setPower(-rawDriveSpeed);
                    backLeft.setPower(rawDriveSpeed);
                }
            }
        }
        else {
            backLeft.setPower(rawDriveSpeed);
            frontRight.setPower(rawDriveSpeed);
        }
    }

    public void updateDirectionFacing( ) {
        updateFieldLocation();

        if (this.config.turnStyle == Config.TANK_TURN) {
            ;
        }

        else {
            if (centerMotor.isBusy()) {
                return; // wheels are not yet in turning position
            }
            //wheels are in turn position, start pivoting
            centerMotor.setPower(0);

            turnSpeed += config.turnSpeedIncrement;
            if (turnSpeed > config.turnSpeed) {
                turnSpeed = config.turnSpeed;
            }
            if (rotateRobotInfo.turnDirection == CCW)
                rawTurnSpeed = turnSpeed * config.turnSpeed * -1;
            else
                rawTurnSpeed = turnSpeed * config.turnSpeed * 1;

            frontRight.setPower(rawTurnSpeed);
            backLeft.setPower(rawTurnSpeed * -1);
        }
    }

    public void beginFacingNewDirection(int direction) {
        rotateRobotInfo.turnDirection = direction;

        // tank drive.. leave wheels as the are.. drive one forward and one backward
        if (this.config.turnStyle == Config.TANK_TURN) {
            ;
        }
        // turn wheels 45 degrees
        else {
            rotateRobotInfo.originalAngle = getAngleFacing();
            updateHeading(45, 0);
        }
        rotateRobotInfo.isUpdateDirectionFacing = true;
    }

    public void stopFacingNewDirection() {
        rotateRobotInfo.isUpdateDirectionFacing = false;
        if (this.config.turnStyle == Config.TANK_TURN) {
            ;
        }
        else {
            double delta = getAngleFacing() - rotateRobotInfo.originalAngle;
            if (rotateRobotInfo.turnDirection == CCW)
                updateHeading(delta, 0);
            else
                updateHeading(-delta, 0);
        }
    }
}