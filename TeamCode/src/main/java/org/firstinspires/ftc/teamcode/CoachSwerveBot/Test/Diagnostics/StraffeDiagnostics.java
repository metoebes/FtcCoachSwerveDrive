package org.firstinspires.ftc.teamcode.CoachSwerveBot.Test.Diagnostics;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;

public class StraffeDiagnostics {
    DataLogger datalog;

    int testCaseStep;
    double angle;
    double magnitude;

    double CCW_ChangeInHeading;
    double CW_ChangeInHeading;

    int centerMotorDirection;
    double turnSpeed;
    double rawTurnSpeed;

    boolean centerMotorIsBusy;
    int centerMotorCurrentPosition;
    int centerMotorTargetPosition;

    int driveDirection;
    double driveSpeed;
    double rawDriveSpeed;
    int driveMotorCurrentPosition;

    boolean marker;

    public StraffeDiagnostics(DataLogger datalog) {
        this.datalog = datalog;
        logHeader();
    }

    public boolean update(
        double time,
        int _testCaseStep,
        double _angle,
        double _magnitude,

        double _CCW_ChangeInHeading,
        double _CW_ChangeInHeading,

        int _centerMotorDirection,
        double _turnSpeed,
        double _rawTurnSpeed,

        boolean _centerMotorIsBusy,
        int _centerMotorCurrentPosition,
        int _centerMotorTargetPosition,

        int _driveDirection,
        double _driveSpeed,
        double _rawDriveSpeed,
        int _driveMotorCurrentPosition,

        boolean _marker) {
        boolean changed = false;

        if (_testCaseStep         != testCaseStep)         { changed = true; testCaseStep = _testCaseStep;}
        if (_angle                != angle)                { changed = true; angle = _angle;}
        if (_magnitude            != magnitude)            { changed = true; magnitude = _magnitude;}
        if (_CCW_ChangeInHeading  != CCW_ChangeInHeading)  { changed = true; CCW_ChangeInHeading = _CCW_ChangeInHeading;}
        if (_CW_ChangeInHeading   != CW_ChangeInHeading)   { changed = true; CW_ChangeInHeading = _CW_ChangeInHeading;}
        if (_centerMotorDirection != centerMotorDirection) { changed = true; centerMotorDirection = _centerMotorDirection;}
        if (_turnSpeed            != turnSpeed)            { changed = true; turnSpeed = _turnSpeed;}
        if (_rawTurnSpeed         != rawTurnSpeed)         { changed = true; rawTurnSpeed = _rawTurnSpeed;}
        if (_centerMotorIsBusy    != centerMotorIsBusy)    { changed = true; centerMotorIsBusy = _centerMotorIsBusy;}
        if (_centerMotorCurrentPosition != centerMotorCurrentPosition)   { changed = true; centerMotorCurrentPosition = _centerMotorCurrentPosition;}
        if (_centerMotorTargetPosition  != centerMotorTargetPosition)    { changed = true; centerMotorTargetPosition  = _centerMotorTargetPosition;}
        if (_driveDirection       != driveDirection)       { changed = true; driveDirection  = _driveDirection;}
        if (_driveSpeed           != driveSpeed)           { changed = true; driveSpeed  = _driveSpeed;}
        if (_rawDriveSpeed        != rawDriveSpeed)        { changed = true; rawDriveSpeed  = _rawDriveSpeed;}
        if (_driveMotorCurrentPosition  != driveMotorCurrentPosition)    { changed = true; driveMotorCurrentPosition  = _driveMotorCurrentPosition;}
        if (_marker               != marker)               { changed = true; marker  = _marker;}

        if (changed) {
            try {
                datalog.addDataLine(
                        time,
                        testCaseStep,
                        angle,
                        magnitude,

                        CCW_ChangeInHeading,
                        CW_ChangeInHeading,

                        centerMotorDirection,
                        turnSpeed,
                        rawTurnSpeed,

                        centerMotorIsBusy,
                        centerMotorCurrentPosition,
                        centerMotorTargetPosition,

                        driveDirection,
                        driveSpeed,
                        rawDriveSpeed,
                        driveMotorCurrentPosition,

                        marker
                );
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return changed;
    }

    public void logHeader() {
        if (datalog == null)
            return;
        try {
            datalog.addDataLine(
                    "time",
                    "step",
                    "joystick angle",
                    "joystick magnitude",
                    "CCW heading change",
                    "CW heading change",
                    "Center Motor Direction",
                    "Center Motor Speed",
                    "Center Motor Raw Speed",
                    "Center Motor isBusy",
                    "Center Motor Position",
                    "Center Motor Target",
                    "Drive Motor Direction",
                    "Drive Motor Speed",
                    "Drive Motor Raw Speed",
                    "Drive Motor Current Position",
                    "marker"
            );
        } catch (IOException e) {
            e.printStackTrace();

        }
    }
}
