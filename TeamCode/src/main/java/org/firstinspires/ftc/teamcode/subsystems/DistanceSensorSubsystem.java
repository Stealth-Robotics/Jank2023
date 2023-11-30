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

        analogLeft = hardwareMap.get(AnalogInput.class, "analogLeft");
        analogRight = hardwareMap.get(AnalogInput.class, "analogRight");

    }


    public double getAnalogLeft(){
        return analogLeft.getVoltage();
    }

    public double getAnalogRight(){
        return analogRight.getVoltage() - 0.3;
    }

    @Override
    public void periodic() {


        FtcDashboard.getInstance().getTelemetry().addData("left dist: ", getAnalogLeft());
        FtcDashboard.getInstance().getTelemetry().addData("right dist: ", getAnalogRight());



        FtcDashboard.getInstance().getTelemetry().update();
    }
}
