package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneSubsystem extends SubsystemBase {

    private final Servo plane;

    public PlaneSubsystem(HardwareMap hardwareMap){
        plane = hardwareMap.get(Servo.class, "plane");
    }

    public void setPlane(double pos){
        plane.setPosition(pos);
    }
}
