package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private DcMotor motor1;
    private DcMotor motor2;
    //TODO: Tune PID
    private final PIDFController elevatorPID = new PIDFController(0, 0, 0, 0);

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotor.class, "elevatorMotor1");
        motor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorPID.setTolerance(10);

    }

    public void setSetpoint(double pos) {
        elevatorPID.setSetPoint(pos);
    }

    public boolean atSetpoint() {
        return elevatorPID.atSetPoint();
    }

    @Override
    public void periodic() {
        //todo: set clamp, this will not work currently
        double power = elevatorPID.calculate(MathUtils.clamp(motor1.getCurrentPosition(), 0, 0));
        motor1.setPower(power);
        motor2.setPower(power);
    }
}
