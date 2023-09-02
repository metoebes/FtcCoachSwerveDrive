package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

import com.acmerobotics.dashboard.canvas.Canvas;

public class DriveWheelPose {
    public double wheelAngle_radians;
    public int ticks = 0; // since last update of position
    public Position positionInInches;
    public boolean reverseDirection;
    public double wheelRadiusInches;
    public String color;

    public DriveWheelPose(double x, double y, double wheelRadiusInches, boolean reverseDirection, String color) {
        this.positionInInches = new Position(x,y);
        this.reverseDirection = reverseDirection;
        this.wheelRadiusInches = wheelRadiusInches;
        this.color = color;
    }

};