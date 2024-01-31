package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Vision.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.teamcode.drive.Vision.PropDetectionPipelineRedClose;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "StickObserverTest")
@Config
public class OpenCV extends LinearOpMode {
    PropDetectionPipelineBlueClose propDetectionRed;
    String webcamName = "Webcam 1";
    private VisionPortal visionPortal2;
    @Override
    public void runOpMode() {
        PropDetectionPipelineBlueClose propDetector = new PropDetectionPipelineBlueClose();
        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(960, 544))
                .addProcessor(propDetector)
                .enableLiveView(true)
                .build();

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.update();
        }
        waitForStart();

    }
}

