package org.firstinspires.ftc.teamcode.Hardware;

public class Pose {

    public Point positionInInches;  //  (x,Y) coordinate
    public double heading_radians;     // angle in the XY plane.. 0 degrees is (x=0)
    public Pose(double x, double y, double heading_radians) {
        this.positionInInches = new Point(x,y);
        this.heading_radians = heading_radians;
    }
};