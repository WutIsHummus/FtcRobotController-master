package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Vision.PlacementPosition;
import org.firstinspires.ftc.teamcode.drive.Vision.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Blue RR Auto Far", group="Linear OpMode")
public class BlueFarAuto extends LinearOpMode {
    PropDetectionPipelineBlueClose propDetectionBlue;
    String webcamName = "Webcam 1";
    private VisionPortal visionPortal2;

    private DcMotorEx liftLeft, liftRight, middleBar, flopper;
    private Servo armGate, box;

    @Override
    public void runOpMode() throws InterruptedException {

        PropDetectionPipelineBlueClose propDetector = new PropDetectionPipelineBlueClose();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(960, 544))
                .addProcessor(propDetector)
                .enableLiveView(true)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        liftLeft = hardwareMap.get(DcMotorEx .class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        middleBar = hardwareMap.get(DcMotorEx.class, "middleBar");
        flopper = hardwareMap.get(DcMotorEx.class, "flopper");
        middleBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleBar.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        flopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        box = hardwareMap.servo.get("box");
        armGate = hardwareMap.servo.get("armGate");

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        drive.setPoseEstimate(new Pose2d(10, 62.5,Math.toRadians(-90)));


        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.update();
        }
        PlacementPosition placementPosition;

        waitForStart();
        if (isStopRequested()) return;

        moveLiftToPosition(-100);
        while(liftRight.isBusy()){if (isStopRequested()) return;}
        moveMiddle(-15);
        box.setPosition(0.15);
        armGate.setPosition(0.2);
        moveMiddle(-300);
        while(middleBar.isBusy()){if (isStopRequested()) return;}
        moveLiftToPosition(0);
        moveMiddle(-250);
        while(middleBar.isBusy()){if (isStopRequested()) return;}

        placementPosition = propDetector.getPlacementPosition();
        TrajectorySequence pixelTraj;
        if (placementPosition == PlacementPosition.LEFT){
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(5)
                    .lineToLinearHeading(new Pose2d(21,42, Math.toRadians(-90)))
                    .addDisplacementMarker(() -> moveFlopper(-150))
                    .waitSeconds(0.1)
                    .back(5)
                    .lineToLinearHeading(new Pose2d(45,41, Math.toRadians(180)))
                    .addDisplacementMarker(() -> moveLiftToPosition(-800))
                    .build();

        }
        else if (placementPosition == PlacementPosition.CENTER){
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(5)
                    .lineToLinearHeading(new Pose2d(13, 33, Math.toRadians(-90)))
                    .addDisplacementMarker(() -> moveFlopper(-150))
                    .waitSeconds(0.1)
                    .back(4)
                    .setTangent(Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(45,35, Math.toRadians(180)))
                    .addDisplacementMarker(() -> moveLiftToPosition(-800))
                    .build();
        }
        else {
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(20)
                    .strafeLeft(13)
                    .turn(Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(10,34) )
                    .addDisplacementMarker(() -> moveFlopper(-150))
                    .waitSeconds(0.1)
                    .lineToLinearHeading(new Pose2d(45,30, Math.toRadians(180)))
                    .addDisplacementMarker(() -> moveLiftToPosition(-800))
                    .build();
        }
        if (pixelTraj != null) drive.followTrajectorySequence(pixelTraj);
        moveMiddle(-1350);
        box.setPosition(0.3);
        while(middleBar.isBusy()){if (isStopRequested()) return;}
        Trajectory nudge = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(4)
                .addDisplacementMarker(2, ()-> armGate.setPosition(0.6))
                .build();
        drive.followTrajectory(nudge);
        moveMiddle(-300);
        box.setPosition(0.3);
        while(middleBar.isBusy()){if (isStopRequested()) return;}
        moveLiftToPosition(0);
        while(liftRight.isBusy()){if (isStopRequested()) return;}
        moveMiddle(0);
        box.setPosition(0);


    }
    private void moveLiftToPosition(int pos) {
        liftLeft.setTargetPosition(pos);
        liftRight.setTargetPosition(pos);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(0.5); // Adjust the power as necessary
        liftRight.setPower(0.5); // Adjust the power as necessary
    }
    private void moveFlopper(int pos){
        flopper.setTargetPosition(pos);
        flopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flopper.setPower(0.5);
    }
    private  void moveMiddle(int pos){
        middleBar.setTargetPosition(pos);
        middleBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleBar.setPower(0.5);
    }
}