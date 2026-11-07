package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Robot-centric TeleOp from the Pedro TeleOp guide. Needs Foresight tuned first;
// before that, drive with AutoTune > Tests > Driving.
//
// gamepad1 drives. gamepad2 runs the mechanisms:
//   right bumper: intake in        left bumper: intake out
//   A: flywheel on/off             X (hold): open stopper once the flywheel is at speed
//   B (hold): match loader         dpad left/right: step turret, dpad up: center turret
@TeleOp(name = "Drive")
public class DriveTeleOp extends OpMode {

    private Robot robot;

    private Command intakeIn;
    private Command intakeOut;
    private Command feed;
    private Command load;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        intakeIn = robot.intake.inCommand();
        intakeOut = robot.intake.outCommand();
        feed = robot.feedCommand();
        load = robot.matchLoader.loadCommand();
    }

    @Override
    public void start() {
        robot.home();

        // Docs sample uses +left_stick_x / +right_stick_x, but Pedro is +lateral = left, +turn = CCW
        // (same negation as Pedro's own Tests > Driving), so stick right must be negated.
        Commands.infinite(() -> robot.follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        )).requiring(robot.follower).schedule();
    }

    @Override
    public void loop() {
        if (gamepad2.rightBumperWasPressed()) {
            intakeIn.schedule();
        } else if (gamepad2.rightBumperWasReleased()) {
            intakeIn.cancel();
        }

        if (gamepad2.leftBumperWasPressed()) {
            intakeOut.schedule();
        } else if (gamepad2.leftBumperWasReleased()) {
            intakeOut.cancel();
        }

        if (gamepad2.aWasPressed()) {
            robot.flywheel.toggleCommand(RobotConstants.FLYWHEEL_SHOOT_RPM).schedule();
        }

        if (gamepad2.xWasPressed()) {
            feed.schedule();
        } else if (gamepad2.xWasReleased()) {
            feed.cancel();
        }

        if (gamepad2.bWasPressed()) {
            load.schedule();
        } else if (gamepad2.bWasReleased()) {
            load.cancel();
        }

        if (gamepad2.dpadLeftWasPressed()) {
            robot.turret.stepCommand(RobotConstants.TURRET_STEP_DEGREES).schedule();
        } else if (gamepad2.dpadRightWasPressed()) {
            robot.turret.stepCommand(-RobotConstants.TURRET_STEP_DEGREES).schedule();
        } else if (gamepad2.dpadUpWasPressed()) {
            robot.turret.centerCommand().schedule();
        }

        robot.update();

        Pose robotPose = robot.follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Flywheel RPM", robot.flywheel.getRpm());
        telemetry.addData("Flywheel Target RPM", robot.flywheel.getTargetRpm());
        telemetry.addData("Flywheel At Speed", robot.flywheel.atSpeed());
        telemetry.addData("Turret Target", robot.turret.getTargetAngle());
    }
}
