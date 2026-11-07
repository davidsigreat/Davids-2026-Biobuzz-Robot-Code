package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

// Measures RobotConstants.FLYWHEEL_KS and FLYWHEEL_KV. Holds each power in POWERS until the wheel settles,
// averages the RPM, then fits power = kS + kV * rpm through the results.
// Takes 24.5 s, so it fits inside the Driver Station's 30 s autonomous timer.
// Runs the flywheel up to POWERS' highest value with no game pieces, so keep clear of the wheel.
@Autonomous(name = "Flywheel SysId", group = "Test")
public class FlywheelSysId extends OpMode {

    private static final double[] POWERS = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    private static final double SETTLE_SECONDS = 2.5;
    private static final double SAMPLE_SECONDS = 1.0;

    private Flywheel flywheel;
    private VoltageSensor battery;
    private final ElapsedTime stepTimer = new ElapsedTime();

    private final double[] averageRpm = new double[POWERS.length];
    private int step = 0;
    private double rpmSum = 0;
    private int sampleCount = 0;
    private double startVoltage;

    private double kS;
    private double kV;
    private double rSquared;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void start() {
        startVoltage = battery.getVoltage();
        stepTimer.reset();
    }

    @Override
    public void loop() {
        if (step < POWERS.length) {
            runStep();
            telemetry.addData("Power", POWERS[Math.min(step, POWERS.length - 1)]);
            telemetry.addData("Flywheel RPM", flywheel.getRpm());
        } else {
            flywheel.stop();
            flywheel.update();
            showResults();
        }
    }

    private void runStep() {
        flywheel.runAtPower(POWERS[step]);

        if (stepTimer.seconds() < SETTLE_SECONDS) {
            return;
        }

        rpmSum += flywheel.getRpm();
        sampleCount++;

        if (stepTimer.seconds() >= SETTLE_SECONDS + SAMPLE_SECONDS) {
            averageRpm[step] = rpmSum / sampleCount;
            rpmSum = 0;
            sampleCount = 0;
            stepTimer.reset();
            step++;

            if (step == POWERS.length) {
                fitGains();
            }
        }
    }

    // Least squares line through (rpm, power): the slope is kV and the intercept is kS.
    private void fitGains() {
        int count = POWERS.length;
        double meanRpm = 0;
        double meanPower = 0;
        for (int i = 0; i < count; i++) {
            meanRpm += averageRpm[i] / count;
            meanPower += POWERS[i] / count;
        }

        double covariance = 0;
        double rpmVariance = 0;
        double powerVariance = 0;
        for (int i = 0; i < count; i++) {
            double rpmError = averageRpm[i] - meanRpm;
            double powerError = POWERS[i] - meanPower;
            covariance += rpmError * powerError;
            rpmVariance += rpmError * rpmError;
            powerVariance += powerError * powerError;
        }

        kV = covariance / rpmVariance;
        kS = meanPower - kV * meanRpm;
        rSquared = covariance * covariance / (rpmVariance * powerVariance);
    }

    private void showResults() {
        // Also catches a wheel that never moved, where rpmVariance is 0 and the fit is NaN.
        if (!(kV > 0)) {
            telemetry.addLine("BAD RESULT: RPM did not rise with power. Check the encoder.");
        }
        telemetry.addData("kS", "%.4f", kS);
        telemetry.addData("kV", "%.7f", kV);
        telemetry.addData("R squared", "%.4f", rSquared);
        telemetry.addData("Battery At Start", "%.2f V", startVoltage);
        for (int i = 0; i < POWERS.length; i++) {
            telemetry.addData("Power " + POWERS[i], "%.0f RPM", averageRpm[i]);
        }
    }
}
