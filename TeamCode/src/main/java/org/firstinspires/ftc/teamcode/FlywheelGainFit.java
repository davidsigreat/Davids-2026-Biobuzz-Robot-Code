package org.firstinspires.ftc.teamcode;

import java.util.List;

// Works out every flywheel gain from Flywheel SysId's measurements. Plain Java, so it can be tested
// without a robot.
//
// The wheel is modelled as a first order system with a delay: power = kS + kV * rpm + kA * rpmPerSecond.
//   kS, kV: straight line through the steady RPM at each power.
//   Time constant and delay: Smith's two point method on the power step (28.3% and 63.2% of the rise).
//   kA: kV * time constant.
//   kP: SIMC rule with the closed loop time constant set equal to the delay.
//   kD: kP * delay / 2, which cancels part of the delay.
// kP and kD are then capped so encoder noise alone can't shake the motor power too much.
public class FlywheelGainFit {

    // The most power that encoder noise is allowed to add through kP or through kD.
    private static final double MAX_NOISE_POWER = 0.03;
    // Samples averaged on each side of a point when finding the step's timing.
    private static final int SMOOTHING_SAMPLES = 2;

    public final double kS;
    public final double kV;
    public final double kA;
    public final double kP;
    public final double kD;
    public final double rampRpmPerSecond;
    public final double rSquared;
    public final double timeConstantSeconds;
    public final double delaySeconds;

    // stepSamples holds {seconds since the step, rpm}, and the step rose from stepStartRpm to stepEndRpm.
    public FlywheelGainFit(double[] powers, double[] averageRpm, double stepStartRpm, double stepEndRpm,
                           List<double[]> stepSamples, double noiseRpm, double loopSeconds, double shootRpm) {
        int count = powers.length;
        double meanRpm = 0;
        double meanPower = 0;
        for (int i = 0; i < count; i++) {
            meanRpm += averageRpm[i] / count;
            meanPower += powers[i] / count;
        }

        double covariance = 0;
        double rpmVariance = 0;
        double powerVariance = 0;
        for (int i = 0; i < count; i++) {
            double rpmError = averageRpm[i] - meanRpm;
            double powerError = powers[i] - meanPower;
            covariance += rpmError * powerError;
            rpmVariance += rpmError * rpmError;
            powerVariance += powerError * powerError;
        }

        kV = covariance / rpmVariance;
        kS = meanPower - kV * meanRpm;
        rSquared = covariance * covariance / (rpmVariance * powerVariance);

        double rise = stepEndRpm - stepStartRpm;
        double t28 = crossingSeconds(stepSamples, stepStartRpm + 0.283 * rise);
        double t63 = crossingSeconds(stepSamples, stepStartRpm + 0.632 * rise);

        timeConstantSeconds = 1.5 * (t63 - t28);
        // The control loop adds one more loop of delay on top of what the step shows.
        delaySeconds = Math.max(t63 - timeConstantSeconds, 0) + loopSeconds;

        kA = kV * timeConstantSeconds;
        double noisePeakRpm = 3 * noiseRpm;
        kP = Math.min(timeConstantSeconds * kV / (2 * delaySeconds), MAX_NOISE_POWER / noisePeakRpm);
        kD = Math.min(kP * delaySeconds / 2, MAX_NOISE_POWER * loopSeconds / (noisePeakRpm * Math.sqrt(2)));
        // The fastest climb the motors can still follow when they reach the shooting speed.
        rampRpmPerSecond = (1 - kS - kV * shootRpm) / kA;
    }

    // Time the smoothed RPM first reaches the given value, or NaN if it never does. Noise on the raw
    // samples would make it cross early, and the delay would read too short.
    private static double crossingSeconds(List<double[]> samples, double rpm) {
        double lastSeconds = 0;
        double lastRpm = 0;
        for (int i = SMOOTHING_SAMPLES; i < samples.size() - SMOOTHING_SAMPLES; i++) {
            double smoothedRpm = 0;
            for (int j = i - SMOOTHING_SAMPLES; j <= i + SMOOTHING_SAMPLES; j++) {
                smoothedRpm += samples.get(j)[1] / (2 * SMOOTHING_SAMPLES + 1);
            }

            double seconds = samples.get(i)[0];
            if (smoothedRpm >= rpm) {
                // Between loops, so the result isn't rounded to a whole loop.
                return lastSeconds + (seconds - lastSeconds) * (rpm - lastRpm) / (smoothedRpm - lastRpm);
            }
            lastSeconds = seconds;
            lastRpm = smoothedRpm;
        }
        return Double.NaN;
    }

    // False if the wheel didn't respond in a way this model can explain. NaN fails every check here.
    public boolean isValid() {
        return kV > 0 && timeConstantSeconds > 0 && rampRpmPerSecond > 0;
    }
}
