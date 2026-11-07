package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointOffsetFit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

// Measures the Pinpoint pod offsets by spinning the robot in place on its own, TURNS times each way.
// PinpointOffsetFit finds how far the tracked point is from the true center of rotation, which is
// the error in the current offsets. Copy the new offsets into pedro/Constants.java and run it again:
// the error should drop to nearly 0. Needs clear space around the robot, and it starts on start.
@TeleOp(name = "Pinpoint Offset Tuner", group = "Test")
public class PinpointOffsetTuner extends OpMode {

    private enum Phase { COUNTERCLOCKWISE, PAUSE, CLOCKWISE, DONE }

    private static final int TURNS = 3;
    private static final double TURN_POWER = 0.4;
    // Skipped at the start of each spin, while the robot speeds up.
    private static final double SPIN_UP_RADIANS = Math.PI / 2;
    private static final double PAUSE_SECONDS = 1.5;
    // Stops the motors if the robot isn't turning, such as when the heading isn't reading.
    private static final double SPIN_TIMEOUT_SECONDS = 30;

    private Follower follower;
    private final ElapsedTime phaseTimer = new ElapsedTime();

    private Phase phase = Phase.COUNTERCLOCKWISE;
    private final List<double[]> samples = new ArrayList<>();
    private double lastHeading = 0;
    private double turnedRadians = 0;
    private boolean timedOut = false;

    private PinpointOffsetFit counterclockwiseFit;
    private PinpointOffsetFit clockwiseFit;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
    }

    @Override
    public void start() {
        follower.setPose(Pose.zero());
        phaseTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        switch (phase) {
            case COUNTERCLOCKWISE:
                if (spinFinished(TURN_POWER)) {
                    counterclockwiseFit = new PinpointOffsetFit(samples);
                    startNextPhase(Phase.PAUSE);
                }
                break;
            case PAUSE:
                follower.manual(0, 0, 0);
                if (phaseTimer.seconds() >= PAUSE_SECONDS) {
                    startNextPhase(Phase.CLOCKWISE);
                }
                break;
            case CLOCKWISE:
                if (spinFinished(-TURN_POWER)) {
                    clockwiseFit = new PinpointOffsetFit(samples);
                    startNextPhase(Phase.DONE);
                }
                break;
            default:
                follower.manual(0, 0, 0);
                showResults();
                return;
        }

        telemetry.addData("Phase", phase);
        telemetry.addData("Turned", "%.0f degrees", Math.toDegrees(turnedRadians));
        telemetry.addData("Samples", samples.size());
    }

    // Spins and collects samples. True once this direction has done its full turns.
    private boolean spinFinished(double power) {
        follower.manual(0, 0, power);

        Pose pose = follower.pose();
        double change = pose.heading() - lastHeading;
        // Heading wraps at a full turn, so take the short way around.
        change -= 2 * Math.PI * Math.round(change / (2 * Math.PI));
        turnedRadians += Math.abs(change);
        lastHeading = pose.heading();

        if (turnedRadians >= SPIN_UP_RADIANS) {
            samples.add(new double[] {phaseTimer.seconds(), pose.x(), pose.y(), pose.heading()});
        }

        if (phaseTimer.seconds() >= SPIN_TIMEOUT_SECONDS) {
            timedOut = true;
            return true;
        }
        return turnedRadians >= SPIN_UP_RADIANS + 2 * Math.PI * TURNS;
    }

    private void startNextPhase(Phase next) {
        phase = next;
        samples.clear();
        turnedRadians = 0;
        lastHeading = follower.pose().heading();
        phaseTimer.reset();
    }

    private void showResults() {
        if (timedOut) {
            telemetry.addLine("BAD RESULT: a spin timed out. Check the drive motors and that heading changes.");
        }

        // Spinning both ways cancels out most of the error that depends on direction.
        double forwardError = (counterclockwiseFit.forwardInches + clockwiseFit.forwardInches) / 2;
        double leftError = (counterclockwiseFit.leftInches + clockwiseFit.leftInches) / 2;
        double currentX = Constants.localizerConfig.xPodOffset.get();
        double currentY = Constants.localizerConfig.yPodOffset.get();

        // The tracked point moves opposite to a change in the offsets, so adding the error cancels it.
        telemetry.addData("New xPodOffset", "%.4f", currentX + leftError);
        telemetry.addData("New yPodOffset", "%.4f", currentY + forwardError);
        telemetry.addData("Current xPodOffset", "%.4f", currentX);
        telemetry.addData("Current yPodOffset", "%.4f", currentY);
        telemetry.addData("Left Error", "%.3f in", leftError);
        telemetry.addData("Forward Error", "%.3f in", forwardError);
        telemetry.addData("Directions Disagree By", "%.3f in", Math.hypot(
                counterclockwiseFit.forwardInches - clockwiseFit.forwardInches,
                counterclockwiseFit.leftInches - clockwiseFit.leftInches));
        telemetry.addData("Fit Error CCW", "%.3f in", counterclockwiseFit.rmsErrorInches);
        telemetry.addData("Fit Error CW", "%.3f in", clockwiseFit.rmsErrorInches);
    }
}
