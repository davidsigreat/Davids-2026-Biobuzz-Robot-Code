package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

// Moves nothing. Sets the robot's pose so TeleOp starts with the right coordinates for testing shots.
// Press start, then stop, and Drive picks up the pose from here. Don't move the robot in between.
@Autonomous(name = "Set Start Pose", preselectTeleOp = "Drive")
public class SetStartPose extends OpMode {

    private static final Pose START_POSE = new Pose(110, 12, Math.PI/2);

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        robot.follower.setPose(START_POSE);
    }

    @Override
    public void loop() {
        // Saves the pose for TeleOp.
        robot.update();

        Pose robotPose = robot.follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}
