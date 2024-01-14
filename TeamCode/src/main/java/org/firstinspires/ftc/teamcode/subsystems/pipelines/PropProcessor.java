package org.firstinspires.ftc.teamcode.subsystems.pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;

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

    static Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(180, 480)
    );

    static Rect RIGHT_RECTANGLE = new Rect(
            new Point(500, 0),
            new Point(640, 480)
    );
    static Rect CENTER_RECT = new Rect(
            new Point(180, 0),
            new Point(500, 480)
    );

    Scalar lowHSVColorUpper;
    Scalar highHSVColorLower;
    Scalar lowHSVColorLower;
    Scalar highHSVColorUpper;
    String defaultPos;


    public PropProcessor(Alliance alliance, String defaultPos){

        this.defaultPos = defaultPos;
        //sets the color thresholds based on alliance
        if(alliance == Alliance.RED){
            lowHSVColorLower = new Scalar(0, 150, 150); //beginning of red
            lowHSVColorUpper = new Scalar(20, 255, 255);

            highHSVColorLower = new Scalar(138, 150, 150); //end of red
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


    public void setRects(Rect left, Rect middle, Rect right){
        LEFT_RECTANGLE = left;
        RIGHT_RECTANGLE = right;
        CENTER_RECT = middle;
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

        if(leftBox < 500000 && centerBox < 500000 && rightBox < 500000 ){
            outStr = defaultPos;
        }
//        FtcDashboard.getInstance().getTelemetry().addData("left", leftBox);
//        FtcDashboard.getInstance().getTelemetry().addData("center", centerBox);
//        FtcDashboard.getInstance().getTelemetry().addData("right", rightBox);
//        FtcDashboard.getInstance().getTelemetry().update();

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
