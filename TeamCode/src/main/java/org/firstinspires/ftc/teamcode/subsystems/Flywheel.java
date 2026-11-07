package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Flywheel {
    // Only this motor has an encoder, so it is the only speed reading.
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private double targetRpm = 0;
    private double currentRpm = 0;
    private double power = 0;

    public Flywheel(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL_LEFT_NAME);
        rightMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL_RIGHT_NAME);
        leftMotor.setDirection(RobotConstants.FLYWHEEL_LEFT_DIRECTION);
        rightMotor.setDirection(RobotConstants.FLYWHEEL_RIGHT_DIRECTION);
        setUp(leftMotor);
        setUp(rightMotor);
    }

    private void setUp(DcMotorEx motor) {
        // update() runs the velocity loop, so the hub's own loop stays off. The encoder still reads.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(RobotConstants.FLYWHEEL_ZERO_POWER);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public void stop() {
        targetRpm = 0;
    }

    // Call once per loop.
    public void update() {
        readRpm();

        if (targetRpm > 0) {
            double feedforward = RobotConstants.FLYWHEEL_KS + RobotConstants.FLYWHEEL_KV * targetRpm;
            double feedback = RobotConstants.FLYWHEEL_KP * (targetRpm - currentRpm);
            // Never below 0: the wheel coasts down rather than being driven backwards.
            applyPower(Range.clip(feedforward + feedback, 0, 1));
        } else {
            applyPower(0);
        }
    }

    // Skips the control loop, for measuring the gains. Call once per loop instead of update().
    public void runAtPower(double openLoopPower) {
        targetRpm = 0;
        readRpm();
        applyPower(Range.clip(openLoopPower, 0, 1));
    }

    private void readRpm() {
        currentRpm = leftMotor.getVelocity() * 60 / RobotConstants.FLYWHEEL_TICKS_PER_REV;
    }

    private void applyPower(double newPower) {
        power = newPower;
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getRpm() {
        return currentRpm;
    }

    public double getPower() {
        return power;
    }

    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(currentRpm - targetRpm) <= RobotConstants.FLYWHEEL_RPM_TOLERANCE;
    }

    public Command spinUpCommand(double rpm) {
        return Commands.instant(() -> setTargetRpm(rpm)).requiring(this);
    }

    public Command stopCommand() {
        return Commands.instant(this::stop).requiring(this);
    }

    public Command toggleCommand(double rpm) {
        return Commands.instant(() -> setTargetRpm(targetRpm > 0 ? 0 : rpm)).requiring(this);
    }
}
