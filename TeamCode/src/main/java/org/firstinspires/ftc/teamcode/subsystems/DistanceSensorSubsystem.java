package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem extends SubsystemBase {


    DistanceSensor rightDistance;
    DistanceSensor leftDistance;
    AnalogInput analogLeft;
    AnalogInput analogRight;




    public DistanceSensorSubsystem(HardwareMap hardwareMap){
        rightDistance = hardwareMap.get(DistanceSensor.class, "distanceRight");
        leftDistance = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        analogLeft = hardwareMap.get(AnalogInput.class, "analogLeft");
        analogRight = hardwareMap.get(AnalogInput.class, "analogRight");

    }

    public double getRightDistanceMillimeters(){
//        return rightFilter.calculate(rightDistance.getDistance(DistanceUnit.MM));
            return rightDistance.getDistance(DistanceUnit.MM);
    }
    public double getLeftDistanceMillimeters(){
//        return leftFilter.calculate(leftDistance.getDistance(DistanceUnit.MM));
        return leftDistance.getDistance(DistanceUnit.MM);

    }

    public double getAnalogLeft(){
        return analogLeft.getVoltage();
    }

    public double getAnalogRight(){
        return analogRight.getVoltage();
    }

    @Override
    public void periodic() {


        FtcDashboard.getInstance().getTelemetry().addData("left distance", getLeftDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().addData("analogDistance", analogLeft.getVoltage());


        FtcDashboard.getInstance().getTelemetry().addData("right distance", getRightDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().addData("Left - Right", getLeftDistanceMillimeters() - getRightDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
