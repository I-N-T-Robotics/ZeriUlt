package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShootInterpolation {

    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    private static final double[][] RPMandDistance = {
        // {46, 2.667},
        // {47, 2.49},
        // {49, 3.06},
        // {50, 3.5},
        // {3000/60, 3.99},
        // {54.1, 4.5},
        // {56, 5.00},
        // {58.7, 5.43},
        // {60, 6},
        // {100, 12},
        {46, 2.49},
        {48, 3.06},
        {49, 3.5},
        {50, 3.99},
        {53.1, 4.5},
        {56, 5.00},
        {58.7, 5.43},
        {60, 6},
        {85, 12},
     
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