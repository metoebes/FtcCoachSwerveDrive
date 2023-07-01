package org.firstinspires.ftc.teamcode.CoachSwerveBot.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.IRobot;
@Disabled
@TeleOp(name = "Coach Base Teleop", group = "coach")
public class CoachBaseTeleOp extends OpMode {
    IRobot robot = null ;

    @Override
    public void init() {
    }

    public void help() {
    }

    @Override
    public void loop() {
    }
    public IRobot newRobot(Class robotClass) {
        try {
            robot = (IRobot) robotClass.newInstance();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        } catch (InstantiationException e) {
            e.printStackTrace();
        }
        return robot;
    }
}
