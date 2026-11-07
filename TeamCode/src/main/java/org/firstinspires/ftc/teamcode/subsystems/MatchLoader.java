package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class MatchLoader {
    private final Servo servo;

    public MatchLoader(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, RobotConstants.MATCH_LOADER_NAME);
    }

    public void load() {
        servo.setPosition(RobotConstants.MATCH_LOADER_LOAD_POSITION);
    }

    public void retract() {
        servo.setPosition(RobotConstants.MATCH_LOADER_RETRACT_POSITION);
    }

    // Stays loaded until cancelled, then retracts.
    public Command loadCommand() {
        return Command.build().setStart(this::load).setEnd(end -> retract()).requiring(this);
    }
}
