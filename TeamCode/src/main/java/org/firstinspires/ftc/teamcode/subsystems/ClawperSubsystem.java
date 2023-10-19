package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawperSubsystem extends SubsystemBase {
    private final Servo clawServo;
    private final Servo rotationServo;



    private int positionCycle = 0;

    public enum ClawperPosition{
        ROTATION_STOW(0.0),
        ROTATION_SCORE(0.0),
        RELEASE_ONE(0.0),
        RELEASE_SECOND(0.0),
        RELEASE_BOTH(0.0),
        CLAW_HOLD(0.0);

        private final double position;
        ClawperPosition(double position){
            this.position = position;
        }

        public double getValue(){
            return position;
        }
    }

    public ClawperSubsystem(HardwareMap hardwareMap){
        clawServo = hardwareMap.get(Servo.class, "claw");
        rotationServo = hardwareMap.get(Servo.class, "rotator");
    }

    public void rotationToPosition(ClawperPosition position){
        rotationServo.setPosition(position.getValue());
    }

    public void clawperRelease(){
        if(positionCycle == 0){
            clawServo.setPosition(ClawperPosition.RELEASE_ONE.getValue());
            positionCycle++;
        }
        if(positionCycle == 1){
            clawServo.setPosition(ClawperPosition.RELEASE_SECOND.getValue());
            positionCycle = 0;
        }
    }

    public void clawperClosedPosition(){
        clawServo.setPosition(ClawperPosition.CLAW_HOLD.getValue());
    }

}
