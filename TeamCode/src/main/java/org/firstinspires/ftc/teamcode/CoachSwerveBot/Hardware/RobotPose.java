package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

public class RobotPose{
    public Position positionInInches;
    public double heading_radians;
    public RobotPose(double x, double y) {
        this.positionInInches = new Position(x,y);
    }
};