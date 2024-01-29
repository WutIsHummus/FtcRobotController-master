package org.firstinspires.ftc.teamcode.drive.CV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
/*@Config*/
public class StickObserverPipeline extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 60;
    public static double strictHighS = 255;
    public StickObserverPipeline() {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }
        List<Mat> hsvChannels = new ArrayList<>();
        Core.split(mat, hsvChannels);

        Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2));
        Core.merge(hsvChannels, mat);
        // Compute average HSV of the input
        Scalar averageMat = Core.mean(mat);

        // Dynamic HSV Threshold Adjustment
        double dynamicLowS = dynamicLowSValue(averageMat);
        double dynamicHighS = dynamicHighSValue(averageMat);
        double dynamicLowV = dynamicLowVValue(averageMat);

        // Dynamic HSV Threshold Adjustment
        Scalar lowHSV1 = new Scalar(0, dynamicLowS, dynamicLowV); // For dark red
        Scalar highHSV1 = new Scalar(10, 255, 255); // Upper bound for dark red

        Scalar lowHSV2 = new Scalar(170, dynamicHighS, dynamicLowV); // For bright red
        Scalar highHSV2 = new Scalar(180, 255, 255); // Upper bound for bright red

        // ...

        // Combine both red thresholds
        Mat thresh1 = new Mat();
        Core.inRange(mat, lowHSV1, highHSV1, thresh1); // Threshold for dark red

        Mat thresh2 = new Mat();
        Core.inRange(mat, lowHSV2, highHSV2, thresh2); // Threshold for bright red

        Mat thresh = new Mat();
        Core.bitwise_or(thresh1, thresh2, thresh); // Combine both thresholds


        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 50); // Adjust these values
        Scalar strictHighHSV = new Scalar(180, strictHighS, 255);
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        double minSizeThreshold = 100; // Set this to the minimum size you're interested in

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);

            if (area < minSizeThreshold) {
                continue; // Skip this contour as it's too small
            }

            double aspectRatio = (double) boundingRect.width / boundingRect.height;
            double minAspectRatio = 0.9;
            double maxAspectRatio = 1.5;

            if (aspectRatio > minAspectRatio && aspectRatio < maxAspectRatio) {
                // This contour is likely your object of interest
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);
            }
        }
        Rect largestBoundingBox = null;
        double largestArea = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestArea) {
                largestArea = area;
                largestBoundingBox = Imgproc.boundingRect(contour);
            }
        }

        if (largestBoundingBox != null) {
            Imgproc.rectangle(input, largestBoundingBox.tl(), largestBoundingBox.br(), new Scalar(0, 255, 0), 2);
        }

        //release all the data
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        thresh.release();

        return input;
    }
    private double dynamicLowSValue(Scalar average) {
        // Adjust the low S value based on average brightness or saturation
        double averageSaturation = average.val[1];
        return Math.max(20, averageSaturation - 50);
    }

    private double dynamicHighSValue(Scalar average) {
        // Adjust the high S value based on average brightness or saturation
        double averageSaturation = average.val[1];
        return Math.min(255, averageSaturation + 150);
    }

    private double dynamicLowVValue(Scalar average) {
        // New function for dynamic low V value
        double averageValue = average.val[2];
        return Math.max(20, averageValue - 50); // Adjust this based on your needs
    }
}
