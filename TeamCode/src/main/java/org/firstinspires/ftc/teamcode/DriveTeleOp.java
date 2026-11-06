package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

// Robot-centric TeleOp from the Pedro TeleOp guide. Needs Foresight tuned first;
// before that, drive with AutoTune > Tests > Driving.
//
// gamepad1 drives. gamepad2 runs the mechanisms:
//   right bumper: intake in        left bumper: intake out
//   A: flywheel on/off             X (hold): open stopper once the flywheel is at speed
//   B (hold): match loader         dpad left/right: step turret, dpad up: center turret
@TeleOp(name = "Drive")
public class DriveTeleOp extends OpMode {

    private static final double SHOOT_RPM = 3000;
    private static final double TURRET_STEP_DEGREES = 5;

    private Follower follower;
    private Intake intake;
    private Flywheel flywheel;
    private Turret turret;
    private Stopper stopper;
    private MatchLoader matchLoader;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        matchLoader = new MatchLoader(hardwareMap);
    }

    @Override
    public void start() {
        turret.setAngle(0);
    }

    @Override
    public void loop() {
        // Docs sample uses +left_stick_x / +right_stick_x, but Pedro is +lateral = left, +turn = CCW
        // (same negation as Pedro's own Tests > Driving), so stick right must be negated.
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        follower.update();

        if (gamepad2.right_bumper) {
            intake.in();
        } else if (gamepad2.left_bumper) {
            intake.out();
        } else {
            intake.stop();
        }

        if (gamepad2.aWasPressed()) {
            if (flywheel.getTargetRpm() > 0) {
                flywheel.stop();
            } else {
                flywheel.setTargetRpm(SHOOT_RPM);
            }
        }

        if (gamepad2.x && flywheel.atSpeed()) {
            stopper.open();
        } else {
            stopper.close();
        }

        if (gamepad2.b) {
            matchLoader.load();
        } else {
            matchLoader.retract();
        }

        if (gamepad2.dpadLeftWasPressed()) {
            turret.setAngle(turret.getTargetAngle() + TURRET_STEP_DEGREES);
        } else if (gamepad2.dpadRightWasPressed()) {
            turret.setAngle(turret.getTargetAngle() - TURRET_STEP_DEGREES);
        } else if (gamepad2.dpadUpWasPressed()) {
            turret.setAngle(0);
        }

        Pose robotPose = follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Flywheel RPM", flywheel.getRpm());
        telemetry.addData("Flywheel Target RPM", flywheel.getTargetRpm());
        telemetry.addData("Flywheel At Speed", flywheel.atSpeed());
        telemetry.addData("Turret Target", turret.getTargetAngle());
    }
}
