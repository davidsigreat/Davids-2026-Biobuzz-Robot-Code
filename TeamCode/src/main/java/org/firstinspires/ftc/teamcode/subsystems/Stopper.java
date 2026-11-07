package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Stopper {
    private final Servo servo;

    public Stopper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, RobotConstants.STOPPER_NAME);
    }

    public void open() {
        servo.setPosition(RobotConstants.STOPPER_OPEN_POSITION);
    }

    public void close() {
        servo.setPosition(RobotConstants.STOPPER_CLOSED_POSITION);
    }

    // Stays open until cancelled, then closes.
    public Command openCommand() {
        return Command.build().setStart(this::open).setEnd(end -> close()).requiring(this);
    }
}
