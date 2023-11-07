package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem extends SubsystemBase {


    DistanceSensor rightDistance;
    DistanceSensor leftDistance;

    public DistanceSensorSubsystem(HardwareMap hardwareMap){
        rightDistance = hardwareMap.get(DistanceSensor.class, "distanceRight");
        leftDistance = hardwareMap.get(DistanceSensor.class, "distanceLeft");

    }

    public double getRightDistanceMillimeters(){
        return rightDistance.getDistance(DistanceUnit.MM);
    }
    public double getLeftDistanceMillimeters(){
        return leftDistance.getDistance(DistanceUnit.MM);
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("left distance", getLeftDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().addData("right distance", getRightDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
