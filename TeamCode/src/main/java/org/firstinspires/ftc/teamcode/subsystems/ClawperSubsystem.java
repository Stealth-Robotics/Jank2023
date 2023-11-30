package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.BooleanSupplier;

@Config
public final class ClawperSubsystem extends SubsystemBase {
    private final Servo clawServo;
    private final Servo rotationServo;


    private int positionCycle = 0;

    private int rotationCycle = 0;
    public static double TEST_POSITION = 0.45;

    private final BooleanSupplier intakeRunning;


    public static enum ClawperPosition {
        ROTATION_STOW(0.50),
        ROTATION_SCORE(0.65),
        RELEASE_ONE(0.5),
        RELEASE_SECOND(0.68),
        RELEASE_BOTH(0.4),
        CLAW_HOLD(0.8),

        ROTATION_CLIMB(0.8);

        private final double position;

        ClawperPosition(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }

    public ClawperSubsystem(HardwareMap hardwareMap, BooleanSupplier intakeRunning) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        rotationServo = hardwareMap.get(Servo.class, "rotator");

        this.intakeRunning = intakeRunning;
    }

    public void rotationToPosition(ClawperPosition position) {
        rotationServo.setPosition(position.getValue());
    }

    public void clawperToPosition(ClawperPosition position) {
        clawServo.setPosition(position.getValue());
    }

    public void clawperRelease() {
        //sets clawper to position based on whether we have 1 or 2 hexes
        if (positionCycle == 0) {
            clawServo.setPosition(ClawperPosition.RELEASE_ONE.getValue());
            positionCycle++;
        }
        else if (positionCycle == 1) {
            clawServo.setPosition(ClawperPosition.RELEASE_SECOND.getValue());
            positionCycle = 0;
        }
    }

    public void rotatinToggle() {
        //sets clawper to position based on whether we have 1 or 2 hexes
        if (rotationCycle == 0) {
            rotationServo.setPosition(ClawperPosition.ROTATION_STOW.getValue());
            rotationCycle++;
            return;
        }
        if (rotationCycle == 1) {
            rotationServo.setPosition(ClawperPosition.ROTATION_SCORE.getValue());
            rotationCycle = 0;
        }
    }

    public void clawperClosedPosition() {
        clawperToPosition(ClawperPosition.CLAW_HOLD);
        positionCycle = 0;
    }

    public void rotationTest(){
        clawServo.setPosition(TEST_POSITION);
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("cycle: ", positionCycle);
        FtcDashboard.getInstance().getTelemetry().update();

        if(intakeRunning.getAsBoolean())
        {
            clawperClosedPosition();
        }
    }
}
