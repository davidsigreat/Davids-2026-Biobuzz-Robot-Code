package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchLoader;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

// Everything TeleOp and Auto share: the subsystems, the Ivy update loop, and commands that use more
// than one subsystem. Don't call setStart/setEnd/requiring on an Ivy group: it replaces the group's logic.
public class Robot {
    public final Follower follower;
    public final Intake intake;
    public final Flywheel flywheel;
    public final Turret turret;
    public final Stopper stopper;
    public final MatchLoader matchLoader;

    public Robot(HardwareMap hardwareMap) {
        // The scheduler is static, so commands from the last OpMode would otherwise still be in it.
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        stopper = new Stopper(hardwareMap);
        matchLoader = new MatchLoader(hardwareMap);
    }

    // Call from start(), not init(), so nothing moves before the match starts.
    public void home() {
        stopper.close();
        matchLoader.retract();
        turret.setAngle(0);
    }

    // Call once per loop.
    public void update() {
        Scheduler.execute();
        flywheel.update();
        follower.update();
    }

    // Holds the stopper open only while the flywheel is at speed, then closes it when cancelled.
    public Command feedCommand() {
        return Commands.infinite(() -> {
            if (flywheel.atSpeed()) {
                stopper.open();
            } else {
                stopper.close();
            }
        }).setEnd(end -> stopper.close()).requiring(stopper);
    }

    // Fires one game piece each time the flywheel is at speed. The flywheel must already be spinning,
    // or this waits forever. Runs the intake to push game pieces up to the stopper.
    public Command shootCommand(int shots) {
        Command oneShot = Groups.sequential(
                Commands.waitUntil(flywheel::atSpeed),
                Groups.deadline(Commands.waitMs(RobotConstants.SHOOT_FEED_MS), stopper.openCommand())
        );
        return Groups.deadline(Groups.repeat(oneShot, shots), intake.inCommand());
    }
}
