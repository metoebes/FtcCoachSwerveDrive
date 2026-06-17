package org.firstinspires.ftc.teamcode.Hardware;

public abstract class GeometryUtilities {

    public static void rotatePoints(Point[] points, double angle) {
        for (int ii=0; ii<points.length; ii++) {
            points[ii].rotate(angle);
        }
    }
    public static void translatePoints(Point[] points, Point offset) {
        for (int ii=0; ii<points.length; ii++) {
            points[ii].translate(offset.x, offset.y);
        }
    }
    public static Point[] copyPoints(Point[] points) {
        Point[] copy = new Point[points.length];
        for (int ii=0; ii<points.length; ii++) {
            copy[ii] = new Point( points[ii].x, points[ii].y);
        }
        return copy;
    }

    public static void rotateTranslatePoints(Point points[], double angle_radians, Point offset) {
        rotatePoints(points, angle_radians);
        translatePoints(points, offset);
    }
    public static void scaleRotateTranslatePoints(Point points[], double scale, double angle_radians, Point offset) {
        for (int ii=0; ii<points.length; ii++) {
            points[ii].x = points[ii].x * scale;
            points[ii].y = points[ii].y * scale;
        }
        rotateTranslatePoints(points, angle_radians, offset);
    }
}
