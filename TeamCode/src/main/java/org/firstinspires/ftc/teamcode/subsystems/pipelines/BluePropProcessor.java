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

public class BluePropProcessor extends ProcessorBase {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();

    double blueThreshold = 0.5;

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


        //hue range for blue
        Scalar lowHSVBlueLower = new Scalar(100, 100, 20);
        Scalar lowHSVBlueUpper = new Scalar(130, 255, 255);

        //maps white to everything in blue range and sets everything else to black
        Core.inRange(testMat, lowHSVBlueLower, lowHSVBlueUpper, lowMat);

        testMat.release();

        finalMat = lowMat;

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]




        if(averagedLeftBox > blueThreshold){        //Must Tune Blue Threshold
            outStr = "left";
        }else if(averagedRightBox> blueThreshold){
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
