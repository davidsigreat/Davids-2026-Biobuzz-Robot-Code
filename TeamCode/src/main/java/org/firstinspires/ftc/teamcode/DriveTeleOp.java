package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Robot-centric TeleOp from the Pedro TeleOp guide. Needs Foresight tuned first;
// before that, drive with AutoTune > Tests > Driving.
//
// Everything is on gamepad1. The sticks drive, and the rest runs the mechanisms:
//   left trigger (hold): intake    left bumper (hold): outtake
//   right trigger (hold): shoot once the flywheel is at speed
//   right bumper: match loader on/off
//   A: flywheel on/off             Y: turret auto aim on/off
//   dpad left/right: step turret, dpad up: center turret. Both turn auto aim off.
//   back (share on PS): reset the pose, with the robot pushed square into a corner
// During init: dpad left picks blue, dpad right picks red.
@TeleOp(name = "Drive")
public class DriveTeleOp extends OpMode {

    private Robot robot;

    private Command intakeIn;
    private Command intakeOut;
    private Command feed;
    private Command load;
    private Command aim;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        intakeIn = robot.intake.inCommand();
        intakeOut = robot.intake.outCommand();
        feed = robot.feedCommand();
        load = robot.matchLoader.loadCommand();
        aim = robot.aimCommand(true);
    }

    @Override
    public void init_loop() {
        Robot.selectAlliance(gamepad1, telemetry);
    }

    @Override
    public void start() {
        robot.home();
        aim.schedule();

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
        if (gamepad1.leftTriggerWasPressed()) {
            intakeIn.schedule();
        } else if (gamepad1.leftTriggerWasReleased()) {
            intakeIn.cancel();
        }

        if (gamepad1.leftBumperWasPressed()) {
            intakeOut.schedule();
        } else if (gamepad1.leftBumperWasReleased()) {
            intakeOut.cancel();
        }

        if (gamepad1.aWasPressed()) {
            robot.flywheel.toggleCommand(robot.shotRpm()).schedule();
        }

        if (gamepad1.yWasPressed()) {
            if (aim.isScheduled()) {
                aim.cancel();
            } else {
                aim.schedule();
            }
        }

        if (gamepad1.backWasPressed()) {
            robot.resetPoseToCorner();
            gamepad1.rumble(300);
        }

        if (gamepad1.rightTriggerWasPressed()) {
            feed.schedule();
        } else if (gamepad1.rightTriggerWasReleased()) {
            feed.cancel();
        }

        if (gamepad1.rightBumperWasPressed()) {
            if (load.isScheduled()) {
                load.cancel();
            } else {
                load.schedule();
            }
        }

        if (gamepad1.dpadLeftWasPressed()) {
            robot.turret.stepCommand(RobotConstants.TURRET_STEP_DEGREES).schedule();
        } else if (gamepad1.dpadRightWasPressed()) {
            robot.turret.stepCommand(-RobotConstants.TURRET_STEP_DEGREES).schedule();
        } else if (gamepad1.dpadUpWasPressed()) {
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
        telemetry.addData("Auto Aim", aim.isScheduled());
        telemetry.addData("Tracking Goal", robot.goalNumber());
        telemetry.addData("Goal In Turret Range", robot.goalInTurretRange());
        telemetry.addData("Distance To Goal", robot.distanceToGoal());
    }
}
