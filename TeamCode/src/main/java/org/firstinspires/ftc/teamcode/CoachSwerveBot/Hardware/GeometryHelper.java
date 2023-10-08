package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

public abstract class GeometryHelper {
    public static void rotateTranslatePoints(Position points[], double angle_radians, Position offset) {
        for (int ii=0; ii<points.length; ii++) {
            points[ii].rotate(angle_radians);
        }
        for (int ii=0; ii<points.length; ii++) {
            points[ii].translate(offset.x, offset.y);
        }
    }
}
