package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

// Tests each mechanism without the drivetrain or Pedro, so it works before Foresight is tuned.
// Everything is on gamepad1:
//   right bumper: intake in        left bumper: intake out
//   A: flywheel on/off             dpad up/down: change flywheel test RPM
//   X (hold): open stopper         B (hold): match loader
//   dpad left/right: step turret   Y: center turret
@TeleOp(name = "Mechanism Test", group = "Test")
public class MechanismTest extends OpMode {

    private static final double START_RPM = 1000;
    private static final double MAX_RPM = 6000;
    private static final double RPM_STEP = 250;
    private static final double TURRET_STEP_DEGREES = 5;

    private Intake intake;
    private Flywheel flywheel;
    private Turret turret;
    private Stopper stopper;
    private MatchLoader matchLoader;

    private double testRpm = START_RPM;
    private boolean flywheelOn = false;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        matchLoader = new MatchLoader(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            intake.in();
        } else if (gamepad1.left_bumper) {
            intake.out();
        } else {
            intake.stop();
        }

        if (gamepad1.dpadUpWasPressed()) {
            testRpm = Math.min(testRpm + RPM_STEP, MAX_RPM);
        } else if (gamepad1.dpadDownWasPressed()) {
            testRpm = Math.max(testRpm - RPM_STEP, RPM_STEP);
        }

        if (gamepad1.aWasPressed()) {
            flywheelOn = !flywheelOn;
        }

        if (flywheelOn) {
            flywheel.setTargetRpm(testRpm);
        } else {
            flywheel.stop();
        }

        if (gamepad1.x) {
            stopper.open();
        } else {
            stopper.close();
        }

        if (gamepad1.b) {
            matchLoader.load();
        } else {
            matchLoader.retract();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setAngle(turret.getTargetAngle() + TURRET_STEP_DEGREES);
        } else if (gamepad1.dpadRightWasPressed()) {
            turret.setAngle(turret.getTargetAngle() - TURRET_STEP_DEGREES);
        } else if (gamepad1.yWasPressed()) {
            turret.setAngle(0);
        }

        telemetry.addData("Flywheel Test RPM", testRpm);
        telemetry.addData("Flywheel On", flywheelOn);
        telemetry.addData("Flywheel Left RPM", flywheel.getLeftRpm());
        telemetry.addData("Flywheel Right RPM", flywheel.getRightRpm());
        telemetry.addData("Flywheel At Speed", flywheel.atSpeed());
        telemetry.addData("Turret Target", turret.getTargetAngle());
    }
}
