package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.stealthrobotics.library.Alliance;

public class CameraSubsystem extends SubsystemBase {

    private final VisionPortal portal;
    private final Alliance alliance;
    PropProcessor processor;
    public CameraSubsystem(HardwareMap hardwareMap, Alliance alliance){
        this.alliance = alliance;
        processor = new PropProcessor(alliance);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();
    }

    @Override
    public void periodic() {
        telemetry.addData("position: ", processor.getOutStr());
    }


}
