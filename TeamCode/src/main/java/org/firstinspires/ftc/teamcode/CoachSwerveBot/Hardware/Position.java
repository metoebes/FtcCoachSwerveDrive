package org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware;

public class Position {
    double x;
    double y;

    public Position(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public void rotate(double angleInRadians) {
        double cos = Math.cos(angleInRadians);
        double sin = Math.sin(angleInRadians);
        double nx = x * cos - y * sin;
        double ny = y * cos + x * sin;
        x = nx;
        y = ny;
    }

    public void translate(double deltaX, double deltaY) {
        this.x += deltaX;
        this.y += deltaY;
    }
}