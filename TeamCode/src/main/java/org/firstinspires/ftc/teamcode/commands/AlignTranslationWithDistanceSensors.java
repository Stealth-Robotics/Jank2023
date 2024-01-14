package org.firstinspires.ftc.teamcode.commands;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.hardware.Sensor;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
@Config

public class AlignTranslationWithDistanceSensors extends CommandBase {

    public enum SensorSide{
        RIGHT,
        LEFT,
        NONE
    }
    private final DriveSubsystem driveSubsystem;
    private final DistanceSensorSubsystem distanceSensorSubsystem;

    PIDController translationController;

    public static double kP = -0.29, kI = -0.4, kD = -0.05;
    public static double setpoint = 1.89;

    private boolean useDistanceOffset = false;


    private SensorSide side = SensorSide.NONE;



    double averageRight = 0;
    double averageLeft = 0;


    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.01);
        translationController.setSetPoint(setpoint);
        translationController.setIntegrationBounds(-0.2, 0.2);
        useDistanceOffset = true;


        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, double position)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.02);
        translationController.setSetPoint(position);

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, double distance, SensorSide side)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.01);
        translationController.setIntegrationBounds(-0.2, 0.2);
        translationController.setSetPoint(distance);

        this.side = side;
        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    @Override
    public void initialize() {
        if(useDistanceOffset){
            translationController.setSetPoint(setpoint - distanceSensorSubsystem.getDistanceOffset());
        }
    }

    @Override
    public void execute() {
        driveSubsystem.update();
        translationController.setPID(kP, kI, kD);
//        translationController.setSetPoint(1.8 - distanceSensorSubsystem.getDistanceOffset());
        double average = (distanceSensorSubsystem.getAnalogRight() + distanceSensorSubsystem.getAnalogLeft()) / 2;
        if(side != SensorSide.NONE){
            if(side == SensorSide.LEFT){
                average = distanceSensorSubsystem.getAnalogLeft();
            }
            if(side == SensorSide.RIGHT){
                average = distanceSensorSubsystem.getAnalogRight();
            }
        }

        double calculation = translationController.calculate(average);
        double staticPower = calculation > 0 ? DriveConstants.kStatic : -DriveConstants.kStatic;



        driveSubsystem.setMotors(calculation + staticPower);

        telemetry.addData("sp", translationController.getSetPoint());

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return translationController.atSetPoint();
    }
}
