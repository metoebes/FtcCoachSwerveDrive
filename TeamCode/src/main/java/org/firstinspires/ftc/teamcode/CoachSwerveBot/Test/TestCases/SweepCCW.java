package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;

import java.io.IOException;

public class SweepCCW extends AbstractTestCase {
    int angle;

    public SweepCCW(Robot _robot) {
        this.testCaseName = "Sweep CCW";
        this.robot = _robot;
    }

    public boolean update() {
        switch (testCaseStep) {
            case 0: {
                startLogging(testCaseName);
                timer.reset();
                angle = (int)robot.targetDirectionAngle;
                testCaseStep++;
            }
            case 1: {
                if (!robot.centerMotor.isBusy()) {
                    if (angle > 370) {
                        // done with this test.. go to next
                        testCaseStep++;
                    } else {
                        /* change heading by 10 degrees */
                        angle += 10;
                        robot.updateHeading(angle, 0.0);
                    }
                }
                logData();
                break;
            }
            case 2: {
                stoplogging();
                testCaseStep++;
                break;
            }
            case 3: {
                break;
            }
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
