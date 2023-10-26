package org.firstinspires.ftc.teamcode.subsystems.pipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.stealthrobotics.library.Alliance;

public class PropProcessor implements VisionProcessor {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();

    double redThreshold = 0.5;

    String outStr = "left";
    //TODO: tune these values

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(100, 480)
    );

    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(500, 0),
            new Point(640, 480)
    );
    static final Rect CENTER_RECT = new Rect(
            new Point(100, 0),
            new Point(500, 480)
    );

    Scalar lowHSVRedUpper;
    Scalar highHSVRedLower;
    Scalar lowHSVRedLower;
    Scalar highHSVRedUpper;

    public PropProcessor(Alliance alliance){
        if(alliance == Alliance.RED){
            lowHSVRedLower = new Scalar(0, 170, 164); //beginning of red
            lowHSVRedUpper = new Scalar(12.8, 255, 255);

            highHSVRedLower = new Scalar(138, 90, 160); //end of red
            highHSVRedUpper = new Scalar(255, 255, 255);
        }
        else if(alliance == Alliance.BLUE){
            lowHSVRedLower = new Scalar(100, 100, 100);
            lowHSVRedUpper = new Scalar(160, 255, 255);

            highHSVRedLower = new Scalar(160, 255, 255); //end of red
            highHSVRedUpper = new Scalar(160, 255, 255);
        }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        //TODO: tune these values




        //maps white to everything in red range and sets everything else to black
        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, highHSVRedLower, highHSVRedUpper, highMat);

        testMat.release();

        //combines ranges since red is split between 0 and 180
        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];
        double centerBox = Core.sumElems(finalMat.submat(CENTER_RECT)).val[0];

        double max = Math.max(Math.max(leftBox, rightBox), centerBox);
        if(leftBox == max) outStr = "left";
        else if(rightBox == max) outStr = "right";
        else outStr = "center";

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]

        finalMat.copyTo(frame);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public String getOutStr(){
        return outStr;
    }
}
