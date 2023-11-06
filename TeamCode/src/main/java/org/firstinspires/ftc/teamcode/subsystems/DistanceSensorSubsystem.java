package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem extends SubsystemBase {


    DistanceSensor distanceSensor;

    public DistanceSensorSubsystem(HardwareMap hardwareMap){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
    }

    public double getDistanceMillimeters(){
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
}
