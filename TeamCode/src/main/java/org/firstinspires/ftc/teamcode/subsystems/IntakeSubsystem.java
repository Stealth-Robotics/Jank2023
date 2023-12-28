package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor intakeMotor;
    private final Servo heightServo;

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

    public double getIntakeSpeed()
    {
        return intakeMotor.getPower();
    }
}
