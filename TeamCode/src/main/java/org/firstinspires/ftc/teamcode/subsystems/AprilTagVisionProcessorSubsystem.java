package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagVisionProcessorSubsystem extends SubsystemBase {

    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    public AprilTagVisionProcessorSubsystem(HardwareMap hardwareMap){
        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();



    }

    @Override
    public void periodic() {
        AprilTagDetection detection = processor.getFreshDetections().get(0);
        if (detection != null) {
            telemetry.addData("Tag ID: ", detection.id);
            telemetry.addData("Tag Pose: ", detection.metadata.fieldPosition);
            telemetry.addData("distance to tag: ", detection.ftcPose);
        }
    }
}
