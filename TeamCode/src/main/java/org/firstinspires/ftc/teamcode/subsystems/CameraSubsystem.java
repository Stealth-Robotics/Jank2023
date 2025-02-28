package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;
import org.stealthrobotics.library.Alliance;

public class CameraSubsystem extends SubsystemBase {

    private final VisionPortal portal;
    private final Alliance alliance;

    private String position;
    PropProcessor processor;
    public CameraSubsystem(HardwareMap hardwareMap, Alliance alliance, String defaultPos){
        this.alliance = alliance;
        //inits processor based on specified alliance
        processor = new PropProcessor(alliance, defaultPos);

        //sets camera info using processor
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();
        //FtcDashboard.getInstance().startCameraStream(processor, 25);
    }

    public final String getPosition(){
        return position;
    }

    public void stopCamera(){
        portal.stopStreaming();
    }

    public void setRects(Rect left, Rect middle, Rect right){
        processor.setRects(left, middle, right);
    }
    @Override
    public void periodic() {
        telemetry.addData("position: ", processor.getOutStr());
        position = processor.getOutStr();
    }


}
