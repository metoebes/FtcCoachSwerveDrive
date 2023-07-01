package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test;

import java.io.IOException;

public class SweepCW extends TestCase {
    int angle;

    public SweepCW() {
        this.testCaseName = "Sweep CW";
    }

    public boolean update() {
        switch (testCaseStep) {
            case 0: {
                startLogging(testCaseName);
                timer.reset();
                angle = (int)robot.targetDirectionAngle;
                testCaseStep++;
                break;
            }
            case 1: {
                if (!robot.centerMotor.isBusy()) {
                    if (angle < 0) {
                        // done with this test.. go to next
                        testCaseStep++;
                    }
                    else {
                        angle -= 10;
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
