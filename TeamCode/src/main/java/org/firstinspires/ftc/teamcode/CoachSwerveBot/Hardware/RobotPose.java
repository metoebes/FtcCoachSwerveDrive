package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

public class RobotPose {
    public Position positionInInches;  //  (x,Y) coordinate
    public double heading_radians;     // angle in the XY plane.. 0 degrees is (x=0)
    public RobotPose(double x, double y, double heading_radians) {
        this.positionInInches = new Position(x,y);
        this.heading_radians = heading_radians;
    }
};