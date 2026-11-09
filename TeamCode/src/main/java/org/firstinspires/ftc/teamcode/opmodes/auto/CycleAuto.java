package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

// Fires the preloads, intakes along path 1, drives path 2, then fires again. The turret and flywheel
// track the goal from the robot's pose for the whole run.
@Autonomous(name = "Cycle Auto", preselectTeleOp = "Drive")
public class CycleAuto extends OpMode {

    private final AutoPaths paths = new AutoPaths();

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        robot.follower.setPose(paths.start());
        robot.home();
        robot.aimCommand(true).schedule();

        Groups.sequential(
                robot.flywheel.spinUpCommand(robot.shotRpm()),
                robot.shootCommand(RobotConstants.AUTO_SHOT_COUNT),
                Groups.deadline(drive(paths.path1()), robot.intake.inCommand()),
                drive(paths.path2()),
                robot.shootCommand(RobotConstants.AUTO_SHOT_COUNT),
                robot.flywheel.stopCommand()
        ).schedule();
    }

    // Follows the path, then waits for the robot to settle on its end pose. The follow command alone
    // finishes 97.5% of the way along the path, while the robot is still moving. Pedro stops being busy
    // once settled, or after its 100 ms timeout, so this can't get stuck here.
    private Command drive(Path path) {
        return Groups.sequential(
                PedroCommands.follow(robot.follower, path).requiring(robot.follower),
                Commands.waitUntil(() -> !robot.follower.isBusy())
        );
    }

    @Override
    public void loop() {
        robot.update();

        Pose robotPose = robot.follower.pose();
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Flywheel RPM", robot.flywheel.getRpm());
        telemetry.addData("Flywheel Target RPM", robot.flywheel.getTargetRpm());
        telemetry.addData("Flywheel At Speed", robot.flywheel.atSpeed());
        telemetry.addData("Turret Target", robot.turret.getTargetAngle());
        telemetry.addData("Goal In Turret Range", robot.goalInTurretRange());
        telemetry.addData("Distance To Goal", robot.distanceToGoal());
    }
}
