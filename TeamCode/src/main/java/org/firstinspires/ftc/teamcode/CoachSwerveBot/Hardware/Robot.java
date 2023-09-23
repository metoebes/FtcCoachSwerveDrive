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

    public enum AttachmentPoint  { FRONT_RIGHT, BACK_LEFT};

    // using a tetrix encoded 1140 counts per revolution
    public final double TICKS_PER_MOTOR_REV = 1440; // tetrix encoder
    public final double TICKS_PER_ROBOT_REVOLUTION = 5120; // robot heading revolves around circle with wheels at 45 degrees
    public final double MIN_ANGLE_CHANGE = 360.0 / TICKS_PER_MOTOR_REV;
    public final double WHEEL_RADIUS_INCHES = 1.5;
    public final double INCHES_PER_REV = 2 * Math.PI * WHEEL_RADIUS_INCHES ;
    public final double DIAG_DST_BETWEEN_WHEELS_INCHES = 13.0;
    public final double DRIVE_BASE_RADIUS = DIAG_DST_BETWEEN_WHEELS_INCHES / 2.0;

    // Physical Hardware attached to the robot 
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor centerMotor;
    public IMU imu;
    
    // Utility classes 
    public Telemetry telemetry;
    public Config config;
    
    public double turnSpeed;   // remember last turn speed to ramp up
    public double driveSpeed;  // remember last drive speed to ramp up

    public int targetCenterMotorTicks;
    public double targetDirectionAngleInDegrees;

    public int driveDirection = CCW;
    public int centerMotorDirection= CCW;
 
    //Variables related to a change in heading .
    public boolean isUpdateDirectionFacing = false;  // wheels are 45 degrees, drive motors are turning robot to new heading
    public int changeHeadingByTurning;               // drive motors are turning CCW or CW to turn robot to new heading
    public double previousHeadingDegrees;            // Capture previous heading, before robot starts turning to new heading.
                                                     // This is so wheels can be returned to driving in the same directions as
                                                     // before the change in heading started.

    public RobotPose robotPose = new RobotPose(0,0, 0);
    double wheelAngle_radians;
    double wheelTicks;

    public void init(HardwareMap hardwareMap, Config _config, Telemetry _telemetry){

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        // imu.resetDeviceConfigurationForOpMode();

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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard == null) {
            return;
        }
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        drawAxis(canvas);
        drawRobot(canvas);
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

    public void rotateTranslatePoints(Position points[], double angle_radians, Position offset) {
        for (int ii=0; ii<points.length; ii++) {
            points[ii].rotate(angle_radians);
        }
        for (int ii=0; ii<points.length; ii++) {
            points[ii].translate(offset.x, offset.y);
        }
    }
    public void drawWheel(Canvas canvas, AttachmentPoint attachmentPoint) {
        // Create a straight gray line (4" long) representing the motor
        Position motor[] = { new Position(WHEEL_RADIUS_INCHES, 0),
                             new Position(WHEEL_RADIUS_INCHES, 5)
        };
        // with a Green/Gold arrow at the end representing the motor CW/CCW direction
        Position arrow[] = {
                new Position(-WHEEL_RADIUS_INCHES, 0),
                new Position( WHEEL_RADIUS_INCHES, 0),
                new Position(WHEEL_RADIUS_INCHES-1, 1),
                new Position(WHEEL_RADIUS_INCHES-1, -1)
        };

        // Rotate the wheel to the direction it is facing.
        // and then translate it to the attachment point on the robot.
        double r = DRIVE_BASE_RADIUS * Math.sqrt(2.0);
        Position wheelAttachmentPositionInInches;
        double wheelAngleRadians;
        if (attachmentPoint == AttachmentPoint.FRONT_RIGHT) {
            wheelAngleRadians = this.wheelAngle_radians;
            wheelAttachmentPositionInInches = new Position(   r,       -r);
        }
        else {
            wheelAngleRadians = this.wheelAngle_radians + Math.PI;
            wheelAttachmentPositionInInches = new Position(  -r,        r);
        }

        rotateTranslatePoints(motor,wheelAngleRadians, wheelAttachmentPositionInInches);

        canvas.setStroke("Gray");
        canvas.setStrokeWidth(0);
        canvas.strokeLine(motor[0].x, motor[0].y, motor[1].x, motor[1].y);

        // Draw an arrow indicating the direction the wheel is rotating
        if (driveDirection == CCW) {
            wheelAngleRadians = this.wheelAngle_radians;
        }
        else {
            wheelAngleRadians = this.wheelAngle_radians + Math.PI;
        }

        String color;
        if (attachmentPoint == AttachmentPoint.BACK_LEFT) {
            wheelAngleRadians += Math.PI;
            color = "Gold";
        }
        else {
            color = "Green";
        }
        rotateTranslatePoints(arrow, wheelAngleRadians, wheelAttachmentPositionInInches);

        canvas.setStroke(color);
        canvas.setStrokeWidth(0);
        canvas.strokeLine(arrow[0].x, arrow[0].y, arrow[1].x, arrow[1].y);
        canvas.strokeLine(arrow[1].x, arrow[1].y, arrow[2].x, arrow[2].y);
        canvas.strokeLine(arrow[1].x, arrow[1].y, arrow[3].x, arrow[3].y);
    }
    public void drawIMUHeading(Canvas canvas) {
        double r = DRIVE_BASE_RADIUS * Math.sqrt(2.0);
        canvas.setStrokeWidth(0);
        Position imuPointer[] = {
                new Position( 0, 0),
                new Position(r, 0),
                new Position(r-1, 1),
                new Position(r, 0),
                new Position(r-1, -1)
        };
        telemetry.addData("IMU Heading", getImuHeadingInDegrees());

        double imuAngleRadian = Math.toRadians(getImuHeadingInDegrees());
        for (int ii=0; ii<imuPointer.length; ii++) {
            imuPointer[ii].rotate(imuAngleRadian);
        }
        for (int ii=0; ii<imuPointer.length; ii++) {
            imuPointer[ii].translate(robotPose.positionInInches.x,  robotPose.positionInInches.y);
        }
        canvas.strokeLine(imuPointer[0].x, imuPointer[0].y, imuPointer[1].x, imuPointer[1].y);
        canvas.strokeLine(imuPointer[1].x, imuPointer[1].y, imuPointer[2].x, imuPointer[2].y);
        canvas.strokeLine(imuPointer[1].x, imuPointer[1].y, imuPointer[3].x, imuPointer[3].y);
        canvas.strokeLine(imuPointer[3].x, imuPointer[3].y, imuPointer[4].x, imuPointer[4].y);
    }

    public void drawRobot(Canvas canvas) {
        double r = DRIVE_BASE_RADIUS * Math.sqrt(2.0);
        Position polygon[] = {
                new Position(r + 1, 0), // 0: point
                new Position(   r,     -1), // 1:
                new Position(   r,       -r), // 2: corner FRONT RIGHT
                new Position(  -r,       -r), // 3: corner BACK RIGHT
                new Position(  -r,        r), // 4: corner BACK LEFT
                new Position(   r,        r), // 5: corner FRONT LEFT
                new Position(   r,      1), // 6:
        };
        for (int ii=0; ii<polygon.length; ii++) {
            polygon[ii].rotate(robotPose.heading_radians);
        }
        for (int ii=0; ii<polygon.length; ii++) {
            polygon[ii].translate(robotPose.positionInInches.x,  robotPose.positionInInches.y);
        }

        // diagonal
        canvas.setStroke("Gray");
        canvas.setStrokeWidth(1);
        canvas.strokeLine(polygon[2].x, polygon[2].y, polygon[4].x, polygon[4].y);
        canvas.strokeLine(polygon[5].x, polygon[5].y, polygon[3].x, polygon[3].y);

        // front face
        canvas.strokeLine(polygon[5].x, polygon[5].y, polygon[6].x, polygon[6].y);
        canvas.strokeLine(polygon[6].x, polygon[6].y, polygon[0].x, polygon[0].y);
        canvas.strokeLine(polygon[0].x, polygon[0].y, polygon[1].x, polygon[1].y);
        canvas.strokeLine(polygon[1].x, polygon[1].y, polygon[2].x, polygon[2].y);

        // imu heading
        canvas.setStroke("Black");
        drawIMUHeading(canvas);

        drawWheel(canvas, AttachmentPoint.FRONT_RIGHT);
        drawWheel(canvas, AttachmentPoint.BACK_LEFT);
    }

    double CenterTickstoDegrees(int ticks) {
        return ticks / (TICKS_PER_MOTOR_REV ) * 360.0;
    }
    int toTicks( double angle) {
        return (int) (angle /360 *(TICKS_PER_MOTOR_REV ));
    }
    public boolean isTurningToNewHeading() {
        return isUpdateDirectionFacing == true;
    }
    public double getImuHeadingInDegrees() {
        YawPitchRollAngles orientation = this.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    public void updateRobotLocation() {

        double currentTicks = frontRight.getCurrentPosition();

        double currentWheelAngle_degrees = CenterTickstoDegrees(centerMotor.getCurrentPosition());
        double currentWheelAngleRadians = Math.toRadians(currentWheelAngle_degrees);

        // going straight
        if (!isUpdateDirectionFacing) {
            // BL and RF ticks traveled should be equal if we are going straight.
            double traveled_inches = (currentTicks- wheelTicks) / (TICKS_PER_MOTOR_REV * INCHES_PER_REV);

            robotPose.positionInInches.x += traveled_inches * Math.cos(currentWheelAngleRadians);
            robotPose.positionInInches.y += traveled_inches * Math.sin(currentWheelAngleRadians);
        }
        // robot is turning to a new heading (both wheels at 45 degree angle pivoting around center of robot)
        else {
            double traveledTicks = currentTicks-wheelTicks;
            double traveledRadians = (traveledTicks / TICKS_PER_ROBOT_REVOLUTION) * 2*Math.PI;
            this.robotPose.heading_radians += traveledRadians;
        }
        wheelTicks = currentTicks;
        wheelAngle_radians = currentWheelAngleRadians;
    }

    public void updateFieldLocation() {
        updateRobotLocation();
        drawField();
    }
    // The direction the robot is moving in. Not necessarily the direction it is facing
    public void updateDirection(double desiredAngleInDegrees, double desiredDriveSpeed) {
        // update our understanding of were we are on the field.
        updateFieldLocation();

        // turning at the moment...
        if (isUpdateDirectionFacing) {
            return;
        }
        // Drive speed is always positive. and must be <= 1.0
        if (desiredDriveSpeed >  1.0)
            desiredDriveSpeed = 1.0;

        // check if going in the opposite drive direction gets us there with less turning
        // or if the smallest angle change is to continue in present drive direction
        double currentWheelAngle_degrees = Math.toDegrees(wheelAngle_radians); // same as front right angle
        double forwardDrive_ChangeInHeading =  AngleUtilities.closestAngle(currentWheelAngle_degrees, desiredAngleInDegrees);        // ex: 0       180
        double reverseDrive_ChangeInHeading = AngleUtilities.closestAngle(currentWheelAngle_degrees, desiredAngleInDegrees + 180);   // ex: 180       0

        int previousDriveDirection = driveDirection;
        double changeInDirection;

        // smallest angle change is none or turning positive direction (CCW)
        if (Math.abs(forwardDrive_ChangeInHeading) <= Math.abs(reverseDrive_ChangeInHeading)) {
            changeInDirection = forwardDrive_ChangeInHeading;
            centerMotorDirection = CCW;
            driveDirection = FORWARD;
        }
        // smallest angle change is to change drive direction (CW)
        else {
            changeInDirection = reverseDrive_ChangeInHeading;
            centerMotorDirection = CW;
            driveDirection = BACKWARD;
        }

        // inflection point - changing direction..
        if (previousDriveDirection !=  driveDirection) {
            driveSpeed =  0;
        }

        // if change in direction is less than .24 degrees its not a significant change. (1 tick = .25 degree)
        targetDirectionAngleInDegrees = currentWheelAngle_degrees;
        if (Math.abs(changeInDirection) <= MIN_ANGLE_CHANGE) {
            turnSpeed =0;
        }
        else {
            targetDirectionAngleInDegrees += changeInDirection;
            targetCenterMotorTicks = toTicks(targetDirectionAngleInDegrees);

            // trapezoidal slow down as approaching target angle.
            // ramp down speed when withing 5 degrees of target.
            turnSpeed = config.turnSpeed;
            if (Math.abs(changeInDirection) < 5) {
                turnSpeed *= Math.abs(changeInDirection) / 5;
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
        double rawDriveSpeed = config.driveSpeed * (driveSpeed * driveDirection);
        double rawTurnSpeed = config.turnSpeed * (turnSpeed * centerMotorDirection);

        centerMotor.setPower(rawTurnSpeed);

        if (isUpdateDirectionFacing) {
            if (this.config.turnStyle == Config.TANK_TURN) {
                if (changeHeadingByTurning  == CCW) {
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

    public void updateHeading( ) {
        updateFieldLocation();

        if (centerMotor.isBusy()) {
            return; // wheels are not yet in turning position
        }
        //wheels are in turn position, start pivoting
        centerMotor.setPower(0);

        turnSpeed += config.turnSpeedIncrement;
        if (turnSpeed > config.turnSpeed) {
            turnSpeed = config.turnSpeed;
        }
        double rawTurnSpeed = 0;
        if (changeHeadingByTurning  == CCW)
            rawTurnSpeed = turnSpeed * config.turnSpeed * -1;
        else
            rawTurnSpeed = turnSpeed * config.turnSpeed * 1;

        frontRight.setPower(rawTurnSpeed);
        backLeft.setPower(rawTurnSpeed * -1);
    }

    public void beginChangeHeading(int direction) {
        changeHeadingByTurning  = direction;
        previousHeadingDegrees = Math.toDegrees(robotPose.heading_radians);
        updateDirection(45, 0);

        isUpdateDirectionFacing = true;
    }

    public void endChangeHeading() {
        isUpdateDirectionFacing = false;

        double delta = Math.toDegrees(robotPose.heading_radians) - previousHeadingDegrees;
        if (changeHeadingByTurning  == CCW)
            updateDirection(-delta, 0);
        else
            updateDirection(delta, 0);
    }
}