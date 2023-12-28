package org.firstinspires.ftc.teamcode.commands;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

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
    private final DriveSubsystem driveSubsystem;
    private final DistanceSensorSubsystem distanceSensorSubsystem;

    PIDController translationController;

    public static double kP = -0.29, kI = -0.2, kD = -0.05;
    public static double setpoint = 1.85;

    private boolean useDistanceOffset = false;

    String trustedSensor;




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
        trustedSensor = null;

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, double position)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.02);
        translationController.setSetPoint(position);
        trustedSensor = null;

        addRequirements(driveSubsystem, distanceSensorSubsystem);

    }

    public AlignTranslationWithDistanceSensors(DriveSubsystem driveSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, double distance, String trust)
    {
        this.driveSubsystem = driveSubsystem;
        this.distanceSensorSubsystem = distanceSensorSubsystem;
        translationController.setSetPoint(distance);

        translationController = new PIDController(kP, kI, kD);
        translationController.setTolerance(0.01);
        translationController.setSetPoint(setpoint);
        translationController.setIntegrationBounds(-0.2, 0.2);
        trustedSensor = trust;
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
        translationController.setPID(kP, kI, kD);
//        translationController.setSetPoint(1.8 - distanceSensorSubsystem.getDistanceOffset());
        double average = (distanceSensorSubsystem.getAnalogRight() + distanceSensorSubsystem.getAnalogLeft()) / 2;
        if(trustedSensor != null){
            if(trustedSensor == "left"){
                average = distanceSensorSubsystem.getAnalogLeft();
            }
            if(trustedSensor == "right"){
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
