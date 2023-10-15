package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class DrawUtilities {

    public static void drawAxis(Canvas canvas) {
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
    }

    public static void drawIMUHeading(Canvas canvas, Robot robot, Telemetry telemetry) {
        double r = robot.DRIVE_BASE_RADIUS * Math.sqrt(2.0);
        canvas.setStrokeWidth(0);
        Position imuPointer[] = {
                new Position( 0, 0),
                new Position(r, 0),
                new Position(r-1, 1),
                new Position(r, 0),
                new Position(r-1, -1)
        };
        telemetry.addData("IMU Heading", robot.getImuHeadingInDegrees());

        double imuAngleRadian = Math.toRadians(robot.getImuHeadingInDegrees());
        for (int ii=0; ii<imuPointer.length; ii++) {
            imuPointer[ii].rotate(imuAngleRadian);
        }
        for (int ii=0; ii<imuPointer.length; ii++) {
            imuPointer[ii].translate(robot.pose.positionInInches.x,  robot.pose.positionInInches.y);
        }
        canvas.strokeLine(imuPointer[0].x, imuPointer[0].y, imuPointer[1].x, imuPointer[1].y);
        canvas.strokeLine(imuPointer[1].x, imuPointer[1].y, imuPointer[2].x, imuPointer[2].y);
        canvas.strokeLine(imuPointer[1].x, imuPointer[1].y, imuPointer[3].x, imuPointer[3].y);
        canvas.strokeLine(imuPointer[3].x, imuPointer[3].y, imuPointer[4].x, imuPointer[4].y);
    }

    public static void drawWheel(Canvas canvas, Robot robot, Robot.AttachmentPoint attachmentPoint, String color) {
        // Create a straight gray line (4" long) representing the motor
        Position motor[] = { new Position(Robot.WHEEL_RADIUS_INCHES, 0),
                new Position(Robot.WHEEL_RADIUS_INCHES, 5)
        };
        // with a Green/Gold arrow at the end representing the motor CW/CCW direction
        Position arrow[] = {
                new Position(-Robot.WHEEL_RADIUS_INCHES, 0),
                new Position( Robot.WHEEL_RADIUS_INCHES, 0),
                new Position(Robot.WHEEL_RADIUS_INCHES-1, 1),
                new Position(Robot.WHEEL_RADIUS_INCHES-1, -1)
        };

        // Rotate the wheel to the direction it is facing.
        // and then translate it to the attachment point on the robot.
        double radius = Robot.DRIVE_BASE_RADIUS * Math.sqrt(2.0);
        Position wheelAttachmentPositionInInches;
        double wheelAngleRadians;
        if (attachmentPoint == Robot.AttachmentPoint.FRONT_RIGHT) {
            wheelAngleRadians = robot.prevWheelAngleRadians;
            wheelAttachmentPositionInInches = new Position(   radius,       -radius);
        }
        else {
            wheelAngleRadians = robot.prevWheelAngleRadians + Math.PI;
            wheelAttachmentPositionInInches = new Position(  -radius,        radius);
        }
        wheelAttachmentPositionInInches.x += robot.pose.positionInInches.x;
        wheelAttachmentPositionInInches.y += robot.pose.positionInInches.y;
        GeometryUtilities.rotateTranslatePoints(motor,wheelAngleRadians, wheelAttachmentPositionInInches);

        canvas.setStroke("Gray");
        canvas.setStrokeWidth(0);
        canvas.strokeLine(motor[0].x, motor[0].y, motor[1].x, motor[1].y);

        // Draw an arrow indicating the direction the wheel is rotating
        if (robot.driveDirection == robot.CCW) {
            wheelAngleRadians = robot.prevWheelAngleRadians;
        }
        else {
            wheelAngleRadians = robot.prevWheelAngleRadians + Math.PI;
        }

        if (attachmentPoint == Robot.AttachmentPoint.BACK_LEFT) {
            wheelAngleRadians += Math.PI;
        }

        GeometryUtilities.rotateTranslatePoints(arrow, wheelAngleRadians, wheelAttachmentPositionInInches);

        canvas.setStroke(color);
        canvas.setStrokeWidth(0);
        canvas.strokeLine(arrow[0].x, arrow[0].y, arrow[1].x, arrow[1].y);
        canvas.strokeLine(arrow[1].x, arrow[1].y, arrow[2].x, arrow[2].y);
        canvas.strokeLine(arrow[1].x, arrow[1].y, arrow[3].x, arrow[3].y);
    }
    public static void drawRobot(Canvas canvas, Robot robot, Telemetry telemetry) {
        double r = robot.DRIVE_BASE_RADIUS * Math.sqrt(2.0);
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
            polygon[ii].rotate(robot.pose.heading_radians);
        }
        for (int ii=0; ii<polygon.length; ii++) {
            polygon[ii].translate(robot.pose.positionInInches.x,  robot.pose.positionInInches.y);
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
        drawIMUHeading(canvas, robot, telemetry);

        drawWheel(canvas, robot, Robot.AttachmentPoint.FRONT_RIGHT, "Green");
        drawWheel(canvas, robot, Robot.AttachmentPoint.BACK_LEFT, "Gold");
    }
    public static void drawField(Robot robot, Telemetry telemetry) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard == null) {
            return;
        }
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        drawAxis(canvas);
        drawRobot(canvas, robot, telemetry);
        dashboard.sendTelemetryPacket(packet);
    }
}
