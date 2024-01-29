package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.CV.CVMaster;

@Autonomous(name = "StickObserverTest")
@Config
public class OpenCV extends LinearOpMode {

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        CVMaster cv = new CVMaster(this);
//      call the function to startStreaming
        cv.observeStick();
        waitForStart();
        while (opModeIsActive()) {
        }
//        stopStreaming
        cv.stopCamera();
    }
}

