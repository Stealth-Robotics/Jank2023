package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor;
    private final Servo heightServo;

    public static double testPos = 0.5;

    public final double level5Height = 0.335;
    public final double level4Height = 0.3;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        heightServo = hardwareMap.get(Servo.class, "heightServo");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

    public void setHeight(double height) {
        heightServo.setPosition(height);
    }

    public void testHeight(){
        heightServo.setPosition(testPos);
    }

    public double getIntakeSpeed()
    {
        return intakeMotor.getPower();
    }
}
