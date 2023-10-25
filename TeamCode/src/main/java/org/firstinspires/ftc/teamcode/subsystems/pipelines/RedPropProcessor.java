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

public class RedPropProcessor extends ProcessorBase {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();

    double redThreshold = 0.5;

    String outStr = "Left";
    //TODO: tune these values

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );

    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        //TODO: tune these values

        Scalar lowHSVRedLower = new Scalar(0, 100, 20); //beginning of red
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar highHSVRedLower = new Scalar(160, 100, 20); //end of red
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);


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

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]




        if(averagedLeftBox > redThreshold){        //Must Tune Red Threshold
            outStr = "left";
        }else if(averagedRightBox> redThreshold){
            outStr = "center";
        }else{
            outStr = "right";
        }
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
