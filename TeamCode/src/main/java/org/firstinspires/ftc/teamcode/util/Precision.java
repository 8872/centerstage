package org.firstinspires.ftc.teamcode.util;

import java.math.BigDecimal;

public class Precision {
    public static double[] calculateSlopeAndIntercept(double x1, double y1, double x2, double y2) {
        double[] result = new double[2];
        double slope = (y2 - y1) / (x2 - x1);
        double yIntercept = y1 - slope * x1;
        result[0] = round(slope,4);
        result[1] = round(yIntercept,4);
        return result;
    }
    public static double round(double x, int scale) {
        return round(x, scale, BigDecimal.ROUND_HALF_UP);
    }
    public static double round(double x, int scale, int roundingMethod) {
        try {
            final double rounded = (new BigDecimal(Double.toString(x))
                    .setScale(scale, roundingMethod))
                    .doubleValue();
            return rounded == 0d ? 0d * x : rounded;
        } catch (NumberFormatException ex) {
            if (Double.isInfinite(x)) {
                return x;
            } else {
                return Double.NaN;
            }
        }
    }
}
