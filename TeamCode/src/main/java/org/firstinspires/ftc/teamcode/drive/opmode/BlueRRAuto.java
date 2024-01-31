package org.firstinspires.ftc.teamcode.drive.opmode;

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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

@Autonomous(name="Blue RR Auto", group="Linear OpMode")
public class BlueRRAuto extends LinearOpMode {
    TfodProcessor myTfodProcessor;
    VisionPortal myVisionPortal;
    private DcMotorEx liftLeft, liftRight, middleBar, flopper;
    private Servo armGate, box;

    @Override
    public void runOpMode() {
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
        /*
        middleBar.setTargetPosition(-15);
        middleBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleBar.setPower(0.5);
        box.setPosition(0.15);
        armGate.setPosition(0.2);
        middleBar.setTargetPosition(-1350);
        middleBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleBar.setPower(0.5);
        */
        initTfod();
        drive.setPoseEstimate(new Pose2d(10, 62.5,Math.toRadians(-90)));

        Trajectory forwardLift;
        forwardLift = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(5)
                .addDisplacementMarker(3,() -> moveLiftToPosition(-1250, -1250))
                .build();
        waitForStart();
        if(isStopRequested()) return;
        drive.followTrajectory(forwardLift);
        String result = checkPixel(10000, opModeIsActive());
        TrajectorySequence pixelTraj = null;
        if (Objects.equals(result, "left")){
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(0,() -> moveLiftToPosition(0, 0))
                    .forward(15)
                    .strafeLeft(13)
                    .forward(5)
                    .waitSeconds(1)
                    .back(5)
                    .lineToLinearHeading(new Pose2d(40,40, Math.toRadians(180)))
                    .build();
        }
        else if (Objects.equals(result, "right")){
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(0,() -> moveLiftToPosition(0, 0))
                    .forward(15)
                    .strafeLeft(13)
                    .turn(Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(13,34) )
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(40,30, Math.toRadians(180)))

                    .build();
        }
        else if (Objects.equals(result, "mid")){
            pixelTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(4)
                    .lineToLinearHeading(new Pose2d(16, 34, Math.toRadians(-90)))
                    .waitSeconds(0.5)
                    .back(4)
                    .setTangent(Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(40,35, Math.toRadians(180)))
                    .build();
        }
        if (pixelTraj != null) drive.followTrajectorySequence(pixelTraj);


    }
    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("model_20240127_082516.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("BLUE", "RED"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        myTfodProcessor.setClippingMargins(0,75,0,150);
        myTfodProcessor.setMinResultConfidence((float)0.9);
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();

        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortalBuilder.enableLiveView(true);
        myVisionPortalBuilder.setCameraResolution(new Size(960, 544));
        myVisionPortalBuilder.enableLiveView(true);
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setAutoStopLiveView(false);
        myVisionPortal = myVisionPortalBuilder.build();
    }
    public String checkPixel(long millis, boolean opModeIsActive){
        long startTime = System.currentTimeMillis();
        myVisionPortal.resumeStreaming();
        Recognition myTfodRecognition;
        while (System.currentTimeMillis() < startTime + millis && opModeIsActive) {
            telemetryTfod();
            List<Recognition> currentRecognitions = myTfodProcessor.getRecognitions();
            if (!currentRecognitions.isEmpty()) {
                myTfodRecognition = currentRecognitions.get(0);
                float width  = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                float diff1 = Math.abs(width - 50);
                float diff2 = Math.abs(width - 400);
                float diff3 = Math.abs(width - 745);
                myVisionPortal.stopStreaming();
                if (diff1 <= diff2 && diff1 <= diff3) return  "right";
                else if (diff2 <= diff1 && diff2 <= diff3) return "mid";
                else return "left";
            }
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(100);
        }
        myVisionPortal.stopStreaming();
        return "none";
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        String v = "";
        if (!currentRecognitions.isEmpty()) {
            float width = currentRecognitions.get(0).getWidth();
            float diff1 = Math.abs(width - 50);
            float diff2 = Math.abs(width - 400);
            float diff3 = Math.abs(width - 745);
            myVisionPortal.stopStreaming();
            if (diff1 <= diff2 && diff1 <= diff2) v = "right";
            else if (diff2 <= diff1 && diff2 <= diff3) v = "mid";
            else v = "left";
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop


    }   // end method telemetryTfod()
    private void moveLiftToPosition(int liftLeftPosition, int liftRightPosition) {
        liftLeft.setTargetPosition(liftLeftPosition);
        liftRight.setTargetPosition(liftRightPosition);

        liftLeft.setPower(0.5); // Adjust the power as necessary
        liftRight.setPower(0.5); // Adjust the power as necessary
    }
}