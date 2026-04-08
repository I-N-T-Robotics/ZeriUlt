package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootInterpolation {

    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    private static final double[][] RPMandDistance = {
        {46, 2.667},
        {47, 2.49},//{2640/60, 2.99},
        {3000/60, 3.06},
        {50, 3.5},
        {3000/60, 3.99},
        {56, 5.00},
        {60, 6},
        {80, 12},
        // {2.99, 2640.0},
        // {3.06, 3000.0},
        // {3.99, 3000.0},
        // {5.00, 3300.0}
    };

    static {
        interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
        for (double[] data : RPMandDistance) {
            interpolatingDoubleTreeMap.put(data[1], data[0]);
        }
    }

    public static double getRPM(double distanceInInches) {
        return interpolatingDoubleTreeMap.get(distanceInInches);
    }
}