package org.firstinspires.ftc.teamcode.subsystems.pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.stealthrobotics.library.Alliance;

import java.util.concurrent.atomic.AtomicReference;

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

    Scalar lowHSVColorUpper;
    Scalar highHSVColorLower;
    Scalar lowHSVColorLower;
    Scalar highHSVColorUpper;


    public PropProcessor(Alliance alliance){
        //sets the color thresholds based on alliance
        if(alliance == Alliance.RED){
            lowHSVColorLower = new Scalar(0, 170, 164); //beginning of red
            lowHSVColorUpper = new Scalar(12.8, 255, 255);

            highHSVColorLower = new Scalar(138, 90, 160); //end of red
            highHSVColorUpper = new Scalar(255, 255, 255);
        }

        else if(alliance == Alliance.BLUE){
            lowHSVColorLower = new Scalar(100, 100, 100); //blue thresholding
            lowHSVColorUpper = new Scalar(160, 255, 255);

            highHSVColorLower = new Scalar(160, 255, 255);
            highHSVColorUpper = new Scalar(160, 255, 255);
        }
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {


        //convert to hsv for thresholding
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);


        //maps white to everything in color range and black to everything out of color range
        Core.inRange(testMat, lowHSVColorLower, lowHSVColorUpper, lowMat);
        Core.inRange(testMat, highHSVColorLower, highHSVColorUpper, highMat);

        testMat.release();

        //combines ranges since red is split between 0 and 180
        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        //gets the sum of the pixels in each rectangle
        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];
        double centerBox = Core.sumElems(finalMat.submat(CENTER_RECT)).val[0];

        //finds max of 3 areas and sets the output string to the corresponding area
        double max = Math.max(Math.max(leftBox, rightBox), centerBox);
        if(leftBox == max) outStr = "left";
        else if(rightBox == max) outStr = "right";
        else outStr = "center";


        finalMat.copyTo(frame);
        //frame.copyTo(frame);


        finalMat.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public String getOutStr(){
        return outStr;
    }

}
