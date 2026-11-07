package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Intake {
    private final DcMotorEx intake1;
    private final DcMotorEx intake2;

    public Intake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE2_NAME);
        intake1.setDirection(RobotConstants.INTAKE1_DIRECTION);
        intake2.setDirection(RobotConstants.INTAKE2_DIRECTION);
        setUp(intake1);
        setUp(intake2);
    }

    private void setUp(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(RobotConstants.INTAKE_ZERO_POWER);
    }

    // -100 to 100, positive pulls game pieces in.
    public void setPercent(double percent) {
        double power = Range.clip(percent, -100, 100) / 100;
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public void in() {
        setPercent(RobotConstants.INTAKE_IN_PERCENT);
    }

    public void out() {
        setPercent(RobotConstants.INTAKE_OUT_PERCENT);
    }

    public void stop() {
        setPercent(0);
    }

    // Runs until cancelled, then stops the intake.
    public Command inCommand() {
        return Command.build().setStart(this::in).setEnd(end -> stop()).requiring(this);
    }

    // Runs until cancelled, then stops the intake.
    public Command outCommand() {
        return Command.build().setStart(this::out).setEnd(end -> stop()).requiring(this);
    }
}
