package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.stealthrobotics.library.math.filter.LinearFilter;

public class DistanceSensorSubsystem extends SubsystemBase {


    DistanceSensor rightDistance;
    DistanceSensor leftDistance;
    AnalogInput analogSensor;

    private final LinearFilter rightFilter = LinearFilter.highPass(2, 0.2);
    private final LinearFilter leftFilter = LinearFilter.highPass(2, 0.2);

    private double filterLeft;
    private double filterRight;


    public DistanceSensorSubsystem(HardwareMap hardwareMap){
        rightDistance = hardwareMap.get(DistanceSensor.class, "distanceRight");
        leftDistance = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        analogSensor = hardwareMap.get(AnalogInput.class, "analogDist");

    }

    public double getRightDistanceMillimeters(){
//        return rightFilter.calculate(rightDistance.getDistance(DistanceUnit.MM));
            return rightDistance.getDistance(DistanceUnit.MM);
    }
    public double getLeftDistanceMillimeters(){
//        return leftFilter.calculate(leftDistance.getDistance(DistanceUnit.MM));
        return leftDistance.getDistance(DistanceUnit.MM);

    }

    public double getAnalog(){
        return analogSensor.getVoltage();
    }

    @Override
    public void periodic() {


        FtcDashboard.getInstance().getTelemetry().addData("left distance", getLeftDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().addData("analogDistance", analogSensor.getVoltage());


        FtcDashboard.getInstance().getTelemetry().addData("right distance", getRightDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().addData("Left - Right", getLeftDistanceMillimeters() - getRightDistanceMillimeters());
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
