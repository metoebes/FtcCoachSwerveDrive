package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.TeleOp.Config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: SwerveDrive")
public class SwerveTeleop extends OpMode {
    final public static String CONFIG_FILENAME = "swerveConfig";

    Robot robot = null;
    Config config = null;
    ImprovedGamepad impGamepad1 = null;
    public ElapsedTime gamepadTimer = new ElapsedTime();  // SECONDS is the default resolution for time() method
    public RobotCentricDrive robotCentricDrive;

    public void printWelcome() {
        telemetry.addLine("Before Pressing Start:");
        telemetry.addLine("- Align drive wheels !!!! ");
        telemetry.addLine("- Gold is front right wheel ");
        telemetry.addLine("- Green is back left wheel");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("Source Code:");
        telemetry.addLine("https://github.com/metoebes/FtcCoachSwerveDrive");
    }

    @Override
    public void init() {
        // connect PC to robot controller wifi
        // open PC browser:  http://192.168.43.1:8080/dash
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1", 0.7f);

        config = new Config();
        config.init(CONFIG_FILENAME, telemetry);

        robot = new Robot();
        robot.init(this.hardwareMap, config, telemetry);

        robotCentricDrive = new RobotCentricDrive(robot, config, impGamepad1, telemetry );
        printWelcome();
    }

    @Override
    public void start() {
        robotCentricDrive.start();
    }

    @Override
    public void loop() {
       impGamepad1.update();
       robotCentricDrive.update();
    }

    @Override
    public void stop() {
        robotCentricDrive.stop();
    }
}