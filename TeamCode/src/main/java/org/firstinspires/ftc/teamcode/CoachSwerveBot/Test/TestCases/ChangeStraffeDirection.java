package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.TestCases;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Configuration.Config;
import org.firstinspires.ftc.teamcode.CoachSwerveBot.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.io.IOException;

public class ChangeStraffeDirection extends AbstractTestCase {
    private double angle;

    public final int START_LOGGING = 0;
    public final int EXECUTE_TESTCASE = 1;
    public final int STOP_LOGGING = 2;
    public final int COMPLETED = 3;


    public ChangeStraffeDirection(Robot _robot, Config _config, ImprovedGamepad _impGamepad1, Telemetry _telemetry) {
        super("Robot Straffe Direction", _robot, _config, _impGamepad1, _telemetry);
    }

    public void help() {
        telemetry.clear();
        telemetry.addData("Robot Straffe Direction", "");
        telemetry.addData("- right joy stick", "straffe direction");
        telemetry.addLine("Press X to return to main menu");
    }

    public boolean update() {
        switch (testCaseStep) {
            case START_LOGGING: {
                startLogging(testCaseName);
                help();
                timer.reset();
                testCaseStep++;
                break;
            }
            case EXECUTE_TESTCASE: {
                // right stick active = straffe direction
                angle = impGamepad1.right_stick_angle;
                robot.updateDirection(angle, 0);

                // Press X to end test
                if (impGamepad1.x.isInitialPress()) {
                    testCaseStep++;
                }
                logData();
                break;
            }
            case STOP_LOGGING: {
                stoplogging();
                testCaseStep++;
                break;
            }
            case COMPLETED: {
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
