package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodInterpolation {

    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    private static final double[][] AngleandDistance = {
        {2.667, 0.2},
        {2.49, 0.2},//{2.921, 0.2},
        //{2.99, 0.2},
        {3.06, 0.2},
        {3.99, 0.2},
        {5.00, 0.2},
    };

    static {
        interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
        for (double[] data : AngleandDistance) {
            interpolatingDoubleTreeMap.put(data[1], data[0]);
        }
    }

    public static double getAngle(double distanceInInches) {
        return interpolatingDoubleTreeMap.get(distanceInInches);
    }
}