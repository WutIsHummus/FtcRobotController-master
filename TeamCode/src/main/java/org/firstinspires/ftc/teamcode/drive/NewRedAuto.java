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

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.Vision.PlacementPosition;
import org.firstinspires.ftc.teamcode.drive.Vision.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.teamcode.drive.Vision.PropDetectionPipelineRedClose;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

@Autonomous(name="Red RR Auto", group="Linear OpMode")
public class NewRedAuto extends LinearOpMode {
    PropDetectionPipelineBlueClose propDetectionRed;
    String webcamName = "Webcam 1";
    private VisionPortal visionPortal2;
    private PropDetectionPipelineRedClose propDetector;

    private DcMotorEx liftLeft, liftRight, middleBar, flopper;
    private Servo armGate, box;

    @Override
    public void runOpMode() throws InterruptedException {

        propDetector = new PropDetectionPipelineRedClose();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
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
        flopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        box = hardwareMap.servo.get("box");
        armGate = hardwareMap.servo.get("armGate");

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.setPoseEstimate(new Pose2d(10, -62.5,Math.toRadians(90)));

        Trajectory forwardLift;
        forwardLift = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(5)
                .addDisplacementMarker(3,() -> moveLiftToPosition(-1250, -1250))
                .build();

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.update();
        }
        PlacementPosition placementPosition;

        waitForStart();

        drive.followTrajectory(forwardLift);

        placementPosition = propDetector.getPlacementPosition();
        TrajectorySequence pixelTraj;
        if (placementPosition == PlacementPosition.LEFT){
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(0,() -> moveLiftToPosition(0, 0))
                    .forward(15)
                    .strafeRight(13)
                    .turn(Math.toRadians(90))
                    .lineToConstantHeading(new Vector2d(13,-30) )
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(40,-40, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequenceAsync(pixelTraj);
        }
        else if (placementPosition == PlacementPosition.CENTER){
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(0,() -> moveLiftToPosition(0, 0))
                    .strafeRight(4)
                    .lineToLinearHeading(new Pose2d(16, -34, Math.toRadians(90)))
                    .waitSeconds(0.5)
                    .back(4)
                    .setTangent(Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(40,-35, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequenceAsync(pixelTraj);
        }
        else {
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(0,() -> moveLiftToPosition(0, 0))
                    .forward(15)
                    .strafeRight(13)
                    .forward(5)
                    .waitSeconds(1)
                    .back(5)
                    .lineToLinearHeading(new Pose2d(40,-30, Math.toRadians(180)))
                    .build();
            drive.followTrajectorySequenceAsync(pixelTraj);
        }
        drive.update();

    }
    private void moveLiftToPosition(int liftLeftPosition, int liftRightPosition) {
        liftLeft.setTargetPosition(liftLeftPosition);
        liftRight.setTargetPosition(liftRightPosition);

        liftLeft.setPower(0.5); // Adjust the power as necessary
        liftRight.setPower(0.5); // Adjust the power as necessary
    }
}