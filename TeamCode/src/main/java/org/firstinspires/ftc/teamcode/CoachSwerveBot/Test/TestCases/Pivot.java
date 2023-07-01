package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;

import java.io.IOException;

public class Pivot extends AbstractTestCase {
    double angle;

    public Pivot(Robot _robot) {
        robot = _robot;
        this.testCaseName = "Pivot";
    }

    public boolean update() {
        switch (testCaseStep) {
            case 0: {
                startLogging(testCaseName);
                timer.reset();
                testCaseStep++;
                angle = robot.getAngleFacing() + 90.0;
                break;
            }
            //  turn wheels until they reach the target 45 degree pivot position
            // then drive until facing 90 degrees from starting position
            case 1: {
                robot.updateDirectionFacing(angle);
                if (!robot.isPivoting()) {
                    testCaseStep++;
                }
                //continue to pivot to face target angle
                break;
            }
            case 2: {
                stoplogging();
                testCaseStep++;
            }
            case 3: {
                break;
            }
        }
        if (!isComplete()) {
            logData();
        }
        return isComplete();
    }
    public boolean isComplete() {
        return (testCaseStep == 3);
    }
    public void logData(){
        if (datalog == null)
            return;

        try {
            datalog.addDataLine(
                    timer.time(),
                    testCaseName,
                    testCaseStep,
                    angle,
                    robot.forwardDrive_ChangeInHeading,
                    robot.reverseDrive_ChangeInHeading,
                    robot.centerMotorDirection,
                    robot.targetCenterMotorTicks,
                    robot.centerMotor.getCurrentPosition(),
                    robot.centerMotor.isBusy(),
                    robot.turnSpeed
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
