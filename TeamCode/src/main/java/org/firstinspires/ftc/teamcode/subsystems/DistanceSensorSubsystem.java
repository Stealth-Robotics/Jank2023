package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem extends SubsystemBase {


    DistanceSensor rightDistance;
    DistanceSensor leftDistance;


    AnalogInput analogLeft;
    AnalogInput analogRight;

    private double rightOffset = 0;

    private double distanceOffset = 0;




    public DistanceSensorSubsystem(HardwareMap hardwareMap){

        analogLeft = hardwareMap.get(AnalogInput.class, "analogLeft");
        analogRight = hardwareMap.get(AnalogInput.class, "analogRight");

    }


    public double getAnalogLeft(){
        return analogLeft.getVoltage();
    }

    public double getAnalogRight(){
        if(analogRight.getVoltage() < 2){
            return analogRight.getVoltage() + 0.1 - rightOffset;
        }
        else{
            return analogRight.getVoltage();

        }

    }

    public void incrementRightOffset(double increment){
        this.rightOffset += increment;
    }

    public void incrementDistance(double increment){
        this.distanceOffset += increment;
    }

    public double getDistanceOffset(){
        return distanceOffset;
    }

    public void resetRightOffset(){
        rightOffset = 0;
    }

    public void resetDistanceOffset(){
        distanceOffset = 0;
    }

    @Override
    public void periodic() {


        FtcDashboard.getInstance().getTelemetry().addData("left dist: ", getAnalogLeft());
        FtcDashboard.getInstance().getTelemetry().addData("right dist: ", getAnalogRight());


        telemetry.addData("distance offset", getDistanceOffset());
        telemetry.addData("rotation offset", rightOffset);
        telemetry.update();


        FtcDashboard.getInstance().getTelemetry().update();
    }
}
