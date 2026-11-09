package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

// Moves only the turret, for comparing where it really points against the telemetry.
// 0 is the back of the robot and + is clockwise seen from above. Everything is on gamepad1:
//   dpad right/left: next/previous angle in ANGLES
//   dpad up/down: step by TURRET_STEP_DEGREES
//   Y: back to 0
@TeleOp(name = "Turret Angle Check", group = "Test")
public class TurretAngleCheck extends OpMode {

    private static final double[] ANGLES = {-175, -135, -90, -45, 0, 45, 90, 135, 175};
    private static final int ZERO_INDEX = 4;

    private Turret turret;
    private int angleIndex = ZERO_INDEX;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void start() {
        turret.setAngle(0);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) {
            angleIndex = Math.min(angleIndex + 1, ANGLES.length - 1);
            turret.setAngle(ANGLES[angleIndex]);
        } else if (gamepad1.dpadLeftWasPressed()) {
            angleIndex = Math.max(angleIndex - 1, 0);
            turret.setAngle(ANGLES[angleIndex]);
        } else if (gamepad1.dpadUpWasPressed()) {
            turret.setAngle(turret.getTargetAngle() + RobotConstants.TURRET_STEP_DEGREES);
        } else if (gamepad1.dpadDownWasPressed()) {
            turret.setAngle(turret.getTargetAngle() - RobotConstants.TURRET_STEP_DEGREES);
        } else if (gamepad1.yWasPressed()) {
            angleIndex = ZERO_INDEX;
            turret.setAngle(0);
        }

        telemetry.addData("Turret Target", turret.getTargetAngle());
        telemetry.addData("Servo Position", "%.4f", turret.getServoPosition());
    }
}
