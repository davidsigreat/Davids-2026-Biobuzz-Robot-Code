package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FlywheelGainFit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.ArrayList;
import java.util.List;

// Measures every flywheel gain for RobotConstants, then FlywheelGainFit turns the measurements into gains.
//   Sweep: holds each power from MIN_POWER to MAX_POWER until the wheel settles and averages the RPM.
//   Coast: lets the wheel slow down for the step, because it only coasts down slowly.
//   Step: jumps from STEP_START_POWER to STEP_END_POWER and records how fast the wheel responds.
// Takes 1 to 1.5 minutes. It's a TeleOp because that's past the Driver Station's 30 s autonomous timer.
// Runs the flywheel up to MAX_POWER with no game pieces, so keep clear of the wheel.
@TeleOp(name = "Flywheel SysId", group = "Test")
public class FlywheelSysId extends OpMode {

    private enum Phase { SWEEP, COAST, STEP_START, STEP, DONE }

    private static final double MIN_POWER = 0.2;
    private static final double MAX_POWER = 0.9;
    private static final double POWER_INCREMENT = 0.05;
    private static final double STEP_START_POWER = 0.3;
    private static final double STEP_END_POWER = 0.7;

    private static final double SETTLE_SECONDS = 2.5;
    private static final double STEP_SETTLE_SECONDS = 4.0;
    private static final double SAMPLE_SECONDS = 1.0;
    private static final double COAST_TIMEOUT_SECONDS = 30;

    private Flywheel flywheel;
    private VoltageSensor battery;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime runTimer = new ElapsedTime();

    private double[] powers;
    private double[] averageRpm;
    private final List<double[]> stepSamples = new ArrayList<>();
    private Phase phase = Phase.SWEEP;
    private int sweepIndex = 0;

    private double rpmSum = 0;
    private double rpmSquareSum = 0;
    private int sampleCount = 0;
    private double meanRpm;
    private double noiseRpm;

    private double sweepNoiseRpm;
    private double stepStartRpm;
    private int loopCount = 0;
    private double startVoltage;

    private FlywheelGainFit fit;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();

        int count = (int) Math.round((MAX_POWER - MIN_POWER) / POWER_INCREMENT) + 1;
        powers = new double[count];
        averageRpm = new double[count];
        for (int i = 0; i < count; i++) {
            powers[i] = MIN_POWER + i * POWER_INCREMENT;
        }
    }

    @Override
    public void start() {
        startVoltage = battery.getVoltage();
        phaseTimer.reset();
        runTimer.reset();
    }

    @Override
    public void loop() {
        loopCount++;

        switch (phase) {
            case SWEEP:
                runSweep();
                break;
            case COAST:
                runCoast();
                break;
            case STEP_START:
                flywheel.runAtPower(STEP_START_POWER);
                if (measured(SETTLE_SECONDS)) {
                    stepStartRpm = meanRpm;
                    phase = Phase.STEP;
                }
                break;
            case STEP:
                runStep();
                break;
            default:
                flywheel.stop();
                flywheel.update();
                showResults();
                return;
        }

        telemetry.addData("Phase", phase);
        telemetry.addData("Point", "%d of %d", Math.min(sweepIndex + 1, powers.length), powers.length);
        telemetry.addData("Power", "%.2f", flywheel.getPower());
        telemetry.addData("Flywheel RPM", flywheel.getRpm());
    }

    private void runSweep() {
        flywheel.runAtPower(powers[sweepIndex]);

        if (measured(SETTLE_SECONDS)) {
            averageRpm[sweepIndex] = meanRpm;
            // The last power is the fastest, so it has the most noise to allow for.
            sweepNoiseRpm = noiseRpm;
            sweepIndex++;

            if (sweepIndex == powers.length) {
                phase = Phase.COAST;
            }
        }
    }

    private void runCoast() {
        flywheel.runAtPower(0);

        int startIndex = (int) Math.round((STEP_START_POWER - MIN_POWER) / POWER_INCREMENT);
        if (flywheel.getRpm() <= averageRpm[startIndex] || phaseTimer.seconds() >= COAST_TIMEOUT_SECONDS) {
            phaseTimer.reset();
            phase = Phase.STEP_START;
        }
    }

    private void runStep() {
        flywheel.runAtPower(STEP_END_POWER);
        stepSamples.add(new double[] {phaseTimer.seconds(), flywheel.getRpm()});

        if (measured(STEP_SETTLE_SECONDS)) {
            fit = new FlywheelGainFit(powers, averageRpm, stepStartRpm, meanRpm, stepSamples, sweepNoiseRpm,
                    runTimer.seconds() / loopCount, RobotConstants.FLYWHEEL_SHOOT_RPM);
            phase = Phase.DONE;
        }
    }

    // Waits out settleSeconds, then averages the RPM for SAMPLE_SECONDS. True once, when that finishes.
    private boolean measured(double settleSeconds) {
        double seconds = phaseTimer.seconds();
        if (seconds < settleSeconds) {
            return false;
        }

        double rpm = flywheel.getRpm();
        rpmSum += rpm;
        rpmSquareSum += rpm * rpm;
        sampleCount++;

        if (seconds < settleSeconds + SAMPLE_SECONDS) {
            return false;
        }

        meanRpm = rpmSum / sampleCount;
        noiseRpm = Math.sqrt(Math.max(rpmSquareSum / sampleCount - meanRpm * meanRpm, 0));
        rpmSum = 0;
        rpmSquareSum = 0;
        sampleCount = 0;
        phaseTimer.reset();
        return true;
    }

    private void showResults() {
        if (!fit.isValid()) {
            telemetry.addLine("BAD RESULT: don't use these. Check the encoder and that the wheel spins freely.");
        }
        telemetry.addData("kS", "%.4f", fit.kS);
        telemetry.addData("kV", "%.7f", fit.kV);
        telemetry.addData("kA", "%.7f", fit.kA);
        telemetry.addData("kP", "%.6f", fit.kP);
        telemetry.addData("kD", "%.7f", fit.kD);
        telemetry.addData("Ramp", "%.0f RPM/s", fit.rampRpmPerSecond);
        telemetry.addData("R squared", "%.4f", fit.rSquared);
        telemetry.addData("Time Constant", "%.3f s", fit.timeConstantSeconds);
        telemetry.addData("Delay", "%.3f s", fit.delaySeconds);
        telemetry.addData("Battery At Start", "%.2f V", startVoltage);
        for (int i = 0; i < powers.length; i++) {
            telemetry.addData(String.format("Power %.2f", powers[i]), "%.0f RPM", averageRpm[i]);
        }
    }
}
