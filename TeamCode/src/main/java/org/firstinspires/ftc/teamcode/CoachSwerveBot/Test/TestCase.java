package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;

import java.io.IOException;

public abstract class TestCase {
    public DataLogger datalog = null;
    public int testCaseStep = 0;
    public String testCaseName = "";
    public ElapsedTime timer = new ElapsedTime();
    Robot robot = null;

    public void startLogging(String name) {
        try {
            DataLogger datalog = new DataLogger(name + ".txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void stoplogging() {
        if (datalog!=null) {
            datalog.close();
            datalog = null;
        }
    }
    public boolean update() {
        return true;
    }
    public boolean isComplete() {
        return true;
    }

}
