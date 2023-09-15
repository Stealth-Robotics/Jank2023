package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private DcMotor motor1;
    private DcMotor motor2;
    //TODO: Tune PID
    private final PIDFController elevatorPID = new PIDFController(0, 0, 0, 0);
    private boolean usePID = true;

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

    public double getEncoderPosition() {
        return motor1.getCurrentPosition();
    }

    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    //TODO: Tune PID
    //call setUsePID(true) to use PID
    @Override
    public void periodic() {
        if (usePID) {
            double power = elevatorPID.calculate(getEncoderPosition());
            setPower(power);
        }
    }
}
