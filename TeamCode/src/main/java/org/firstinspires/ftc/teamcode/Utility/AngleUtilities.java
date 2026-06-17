package org.firstinspires.ftc.teamcode.Utility;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AngleUtilities {

    /**
     * Normalizes an angle to be between -180 and 179 degrees
     *
     * @param angle angle to be normalized
     * @return an angle to be between -180 and 179 degrees
     */
    public static double getNormalizedAngle(double angle) {
        return AngleUnit.normalizeDegrees(angle);
    }

    public static double toRadians(double degrees) {
        return getPositiveNormalizedAngle(degrees) * Math.PI / 180.0;
    }
    public static double toDegrees(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;
    }

    /**
     * Normalizes an angle to be between 0 and 359 degrees
     *
     * @param angle angle to be normalized
     * @return an angle between 0 and 359 degrees
     */
    public static double getPositiveNormalizedAngle(double angle) {
        return (getNormalizedAngle(angle) + 360) % 360;
    }

    public static double getRadius(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static double getAngle(double x, double y) {

        double angleRad = Math.atan2(y, x);
        double angleDegrees = AngleUtilities.toDegrees(angleRad);

        return AngleUtilities.getNormalizedAngle(angleDegrees);
    }
    public static double closestAngle(double start, double end) {
        // modulo 360
        start = start - (Math.floor(start/360.0) * 360.0);
        end = end - (Math.floor(end/360.0) * 360.0);
        double dir = end - start;

        if(Math.abs(dir) >  180) {
            dir = -(Math.signum(dir)* 360.0) + dir;
        }
        return dir;
    }
}
