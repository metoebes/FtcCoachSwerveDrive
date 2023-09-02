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
    public final double TICKS_PER_ROBOT_REVOLUTION = 5120; // robot heading revolves around circle with wheels at 45 degrees
    public final double MIN_ANGLE_CHANGE = 360.0 / TICKS_PER_MOTOR_REV;
    public final double WHEEL_RADIUS_INCHES = 1.5;
    public final double INCHES_PER_REV = 2 * Math.PI * WHEEL_RADIUS_INCHES ;
    public final double DIAG_DST_BETWEEN_WHEELS_INCHES = 13.0;
    public final double DRIVE_BASE_RADIUS = DIAG_DST_BETWEEN_WHEELS_INCHES / 2.0;

    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor centerMotor;
    public IMU imu;
    public Telemetry telemetry;
    public Config config;

    public double turnSpeed;   // remember last turn speed to ramp up
    public double driveSpeed;  // remember last drive speed to ramp up

    public int targetCenterMotorTicks;
    public double targetDirectionAngleInDegrees;

    public int driveDirection = CCW;
    public int centerMotorDirection= CCW;

    private class RotateRobotInfo {
        public boolean isUpdateDirectionFacing = false;
        public double previousHeadingDegrees;
        public double previousHeadingTicks;
        public int turnDirection;
    };
    RotateRobotInfo rotateRobotInfo = new RotateRobotInfo();

    public RobotPose robotPose = new RobotPose(0,0);
    DriveWheelPose frontRightWheel = new DriveWheelPose(DRIVE_BASE_RADIUS * Math.sqrt(2.0), -DRIVE_BASE_RADIUS * Math.sqrt(2.0), DRIVE_BASE_RADIUS, false, "gold");
    DriveWheelPose backLeftWheel = new DriveWheelPose( -DRIVE_BASE_RADIUS * Math.sqrt(2.0),  DRIVE_BASE_RADIUS * Math.sqrt(2.0), DRIVE_BASE_RADIUS, true, "green");

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

    public void rotateTranslatePoints(Position points[], double angle_radians, Position offset) {
        for (int ii=0; ii<points.length; ii++) {
            points[ii].rotate(angle_radians);
        }
        for (int ii=0; ii<points.length; ii++) {
            points[ii].translate(offset.x, offset.y);
        }
    }
    public void drawWheel(Canvas canvas, DriveWheelPose wheel) {
        // Create a straight line (4" long) with arrow at the end
        Position motor[] = { new Position(WHEEL_RADIUS_INCHES, 0),
                             new Position(WHEEL_RADIUS_INCHES, 5)
        };
        if (wheel.reverseDirection) {
            rotateTranslatePoints(motor, wheel.wheelAngle_radians + Math.PI, wheel.positionInInches);
        }
        else {
            rotateTranslatePoints(motor, wheel.wheelAngle_radians, wheel.positionInInches);
        }

        canvas.setStroke("gray");
        canvas.setStrokeWidth(0);
        canvas.strokeLine(motor[0].x, motor[0].y, motor[1].x, motor[1].y);

        Position arrow[] = {
                new Position(-WHEEL_RADIUS_INCHES, 0),
                new Position( WHEEL_RADIUS_INCHES, 0),
                new Position(WHEEL_RADIUS_INCHES-1, 1),
                new Position(WHEEL_RADIUS_INCHES-1, -1)
        };
        if (driveDirection == CCW) {
            rotateTranslatePoints(arrow, wheel.wheelAngle_radians, wheel.positionInInches);
        } else {
            rotateTranslatePoints(arrow, wheel.wheelAngle_radians + Math.PI, wheel.positionInInches);
        }
        canvas.setStroke(wheel.color);
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
                new Position(r + 1,0), // 0: point
                new Position(   r,     -1), // 1:
                new Position(   r,       -r), // 2: corner
                new Position(  -r,       -r), // 3: corner
                new Position(  -r,        r), // 4: corner
                new Position(   r,        r), // 5: corner
                new Position(   r,      1), // 6:
        };
        for (int ii=0; ii<polygon.length; ii++) {
            polygon[ii].rotate(robotPose.heading_radians);
        }
        for (int ii=0; ii<polygon.length; ii++) {
            polygon[ii].translate(robotPose.positionInInches.x,  robotPose.positionInInches.y);
        }

        // diagonal
        canvas.setStroke("gray");
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

        drawWheel(canvas, this.frontRightWheel);
        drawWheel(canvas, this.backLeftWheel);
    }

    double CenterTickstoDegrees(int ticks) {
        return ticks / (TICKS_PER_MOTOR_REV ) * 360.0;
    }
    int toTicks( double angle) {
        return (int) (angle /360 *(TICKS_PER_MOTOR_REV ));
    }
    public boolean isTurningToNewHeading() {
        return rotateRobotInfo.isUpdateDirectionFacing == true;
    }
    public double getImuHeadingInDegrees() {
        YawPitchRollAngles orientation = this.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void updateWheelLocation(DriveWheelPose wheel, int current_ticks, double wheelAngle_degrees) {

        double angle_radians = Math.toRadians(wheelAngle_degrees);

        // going straight
        if (!rotateRobotInfo.isUpdateDirectionFacing) {
            double traveled_ticks = current_ticks - wheel.ticks;
            double traveled_inches = traveled_ticks / TICKS_PER_MOTOR_REV * INCHES_PER_REV;
            wheel.positionInInches.x += traveled_inches * Math.cos(angle_radians);;
            wheel.positionInInches.y += traveled_inches * Math.sin(angle_radians);;
        }
        // turning (both wheels at 45 degree angle pivoting around center of robot)
        else {
            double traveled_ticks = frontRight.getCurrentPosition()-rotateRobotInfo.previousHeadingTicks;
            telemetry.addData("traveled Ticks", traveled_ticks);

            telemetry.addData("encoder %", traveled_ticks / TICKS_PER_ROBOT_REVOLUTION);
            double thetaInRadians = (traveled_ticks / TICKS_PER_ROBOT_REVOLUTION) * 2*Math.PI;
            telemetry.addData("Encoder radians", thetaInRadians);
            telemetry.addData("N full circles", Math.floor((thetaInRadians + Math.PI)/(Math.PI*2)));
            thetaInRadians = thetaInRadians - (Math.PI *2.0 * Math.floor((thetaInRadians + Math.PI)/(Math.PI*2)));
            telemetry.addData("Encoder radians N", thetaInRadians);
            this.robotPose.heading_radians = thetaInRadians;

            telemetry.addData("Encoder angle", Math.toDegrees(thetaInRadians));

            // starting from where the wheel is mounted on the circle of rotation.
            // Determine X and Y traveled on perimeter of circle
            double wheelMountedAtRadians = Math.toRadians(45);
            if (wheel.reverseDirection)
                wheelMountedAtRadians += Math.PI;
            double radius = DIAG_DST_BETWEEN_WHEELS_INCHES/2.0;
            double XonCircle = radius * Math.cos(this.robotPose.heading_radians + wheelMountedAtRadians);
            double YonCircle = radius * Math.sin(this.robotPose.heading_radians + wheelMountedAtRadians);

            // translate from [0,0] to robot center position
            wheel.positionInInches.x = robotPose.positionInInches.x + XonCircle;
            wheel.positionInInches.y = robotPose.positionInInches.y + YonCircle;
        }

        wheel.wheelAngle_radians = angle_radians;
        wheel.ticks = current_ticks;
    }
    public void updateRobotLocation() {
        robotPose.positionInInches.x = (backLeftWheel.positionInInches.x + frontRightWheel.positionInInches.x)/2.0;
        robotPose.positionInInches.y = (backLeftWheel.positionInInches.y + frontRightWheel.positionInInches.y)/2.0;
    }
    public void updateFieldLocation() {
        int current_BL_ticks = backLeft.getCurrentPosition();
        int current_FR_ticks = frontRight.getCurrentPosition();
        double currentWheelAngle_degrees = CenterTickstoDegrees(centerMotor.getCurrentPosition());

        updateWheelLocation(this.backLeftWheel, current_BL_ticks, currentWheelAngle_degrees);
        updateWheelLocation(this.frontRightWheel, current_FR_ticks, currentWheelAngle_degrees);
        updateRobotLocation();
        drawField();
    }
    // The direction the robot is moving in. Not necessarily the direction it is facing
    public void updateDirection(double desiredAngleInDegrees, double desiredDriveSpeed) {
        // update our understanding of were we are on the field.
        updateFieldLocation();

        // turning at the moment...
        if (rotateRobotInfo.isUpdateDirectionFacing) {
            return;
        }
        // Drive speed is always positive. and must be <= 1.0
        if (desiredDriveSpeed >  1.0)
            desiredDriveSpeed = 1.0;

        // check if going in the opposite drive direction gets us there with less turning
        // or if the smallest angle change is to continue in present drive direction
        double currentWheelAngle_degrees = Math.toDegrees(backLeftWheel.wheelAngle_radians); // same as front right angle
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
        if (rotateRobotInfo.turnDirection == CCW)
            rawTurnSpeed = turnSpeed * config.turnSpeed * -1;
        else
            rawTurnSpeed = turnSpeed * config.turnSpeed * 1;

        frontRight.setPower(rawTurnSpeed);
        backLeft.setPower(rawTurnSpeed * -1);
        telemetry.addData("ticks traveled",  frontRight.getCurrentPosition()-rotateRobotInfo.previousHeadingTicks);
    }

    public void beginChangeHeading(int direction) {
        rotateRobotInfo.turnDirection = direction;
        rotateRobotInfo.previousHeadingTicks = frontRight.getCurrentPosition();
        rotateRobotInfo.previousHeadingDegrees = Math.toDegrees(robotPose.heading_radians);
        updateDirection(45, 0);

        rotateRobotInfo.isUpdateDirectionFacing = true;
    }

    public void endChangeHeading() {
        rotateRobotInfo.isUpdateDirectionFacing = false;

        double delta = Math.toDegrees(robotPose.heading_radians) - rotateRobotInfo.previousHeadingDegrees;
        if (rotateRobotInfo.turnDirection == CCW)
            updateDirection(-delta, 0);
        else
            updateDirection(delta, 0);
    }
}