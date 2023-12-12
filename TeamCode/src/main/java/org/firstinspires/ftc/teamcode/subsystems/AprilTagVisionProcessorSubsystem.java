package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.AprilTagDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagVisionProcessorSubsystem extends SubsystemBase {

    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    double heading = -1;

    public AprilTagVisionProcessorSubsystem(HardwareMap hardwareMap){
        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagDatabase.getLibrary())
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();



    }

//    public Pose2d robotPoseFromAprilTagDetection(){
//        AprilTagDetection detection = processor.getFreshDetections().get(0);
//        if(detection != null){
//            VectorF aprilTagFieldPosition = detection.metadata.fieldPosition;
//            AprilTagPoseFtc distanceFromTag = detection.ftcPose;
//            Vector2d tagDistance = new Vector2d(distanceFromTag.x, distanceFromTag.y);
//            Vector2d aprilTagPose = new Vector2d(aprilTagFieldPosition.get(0), aprilTagFieldPosition.get(1));
//            Vector2d robotPose = aprilTagPose.minus(tagDistance);
//            return new Pose2d(robotPose.getX(), robotPose.getY(), distanceFromTag.bearing);
//
//        }
//        else return null;
//    }
    public void getHeadingAprilTags(){
        if(processor.getDetections() != null) {
            if (processor.getDetections().size() > 0) {
                for(int i = 0; i < processor.getDetections().size(); i++){
                    if(processor.getDetections().get(i).id == 5){
                        heading = processor.getDetections().get(i).ftcPose.bearing;
                    }
                }

            }
            else{
                heading = -1;
            }
        }

    }

    public double getRotation(){
        return heading;
    }

    @Override
    public void periodic() {
        getHeadingAprilTags();
        FtcDashboard.getInstance().getTelemetry().addData("aprilTagHHeading", heading);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
