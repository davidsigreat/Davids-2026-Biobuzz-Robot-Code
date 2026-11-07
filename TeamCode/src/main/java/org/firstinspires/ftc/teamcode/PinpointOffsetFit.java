package org.firstinspires.ftc.teamcode;

import java.util.List;

// Finds how far the Pinpoint's tracked point is from the true center of rotation. Plain Java, so it can
// be tested without a robot.
//
// While the robot spins in place, a tracked point that isn't on the center of rotation traces a circle:
//   x = centerX + driftX * t + forward * cos(heading) - left * sin(heading)
//   y = centerY + driftY * t + forward * sin(heading) + left * cos(heading)
// Those are linear in the six unknowns, so least squares over every sample solves them directly.
// The drift terms absorb the robot sliding slowly across the floor during the spin.
public class PinpointOffsetFit {
    private static final int UNKNOWNS = 6;
    private static final int FORWARD = 4;
    private static final int LEFT = 5;

    // Where the tracked point is from the center of rotation, in the robot's frame.
    public final double forwardInches;
    public final double leftInches;
    // How far the samples sit from the fitted circle on average. Small means a clean spin.
    public final double rmsErrorInches;

    // Each sample is {seconds, x, y, heading in radians}.
    public PinpointOffsetFit(List<double[]> samples) {
        double[][] normal = new double[UNKNOWNS][UNKNOWNS + 1];
        for (double[] sample : samples) {
            addRow(normal, xRow(sample), sample[1]);
            addRow(normal, yRow(sample), sample[2]);
        }

        double[] solution = solve(normal);
        forwardInches = solution[FORWARD];
        leftInches = solution[LEFT];

        double squaredError = 0;
        for (double[] sample : samples) {
            double errorX = sample[1] - dot(xRow(sample), solution);
            double errorY = sample[2] - dot(yRow(sample), solution);
            squaredError += errorX * errorX + errorY * errorY;
        }
        rmsErrorInches = Math.sqrt(squaredError / samples.size());
    }

    private static double[] xRow(double[] sample) {
        return new double[] {1, 0, sample[0], 0, Math.cos(sample[3]), -Math.sin(sample[3])};
    }

    private static double[] yRow(double[] sample) {
        return new double[] {0, 1, 0, sample[0], Math.sin(sample[3]), Math.cos(sample[3])};
    }

    private static void addRow(double[][] normal, double[] row, double value) {
        for (int i = 0; i < UNKNOWNS; i++) {
            for (int j = 0; j < UNKNOWNS; j++) {
                normal[i][j] += row[i] * row[j];
            }
            normal[i][UNKNOWNS] += row[i] * value;
        }
    }

    private static double dot(double[] row, double[] solution) {
        double sum = 0;
        for (int i = 0; i < UNKNOWNS; i++) {
            sum += row[i] * solution[i];
        }
        return sum;
    }

    // Gaussian elimination with partial pivoting.
    private static double[] solve(double[][] matrix) {
        for (int column = 0; column < UNKNOWNS; column++) {
            int pivot = column;
            for (int row = column + 1; row < UNKNOWNS; row++) {
                if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) {
                    pivot = row;
                }
            }
            double[] swap = matrix[column];
            matrix[column] = matrix[pivot];
            matrix[pivot] = swap;

            for (int row = column + 1; row < UNKNOWNS; row++) {
                double factor = matrix[row][column] / matrix[column][column];
                for (int k = column; k <= UNKNOWNS; k++) {
                    matrix[row][k] -= factor * matrix[column][k];
                }
            }
        }

        double[] solution = new double[UNKNOWNS];
        for (int row = UNKNOWNS - 1; row >= 0; row--) {
            double sum = matrix[row][UNKNOWNS];
            for (int k = row + 1; k < UNKNOWNS; k++) {
                sum -= matrix[row][k] * solution[k];
            }
            solution[row] = sum / matrix[row][row];
        }
        return solution;
    }
}
