package org.usfirst.frc.team1318.robot.common;

public class Helpers
{
    public static double EnforceRange(double value, double minValue, double maxValue)
    {
        if (value > maxValue)
        {
            return maxValue;
        }
        else if (value < minValue)
        {
            return minValue;
        }

        return value;
    }

    public static boolean WithinDelta(double actualValue, double expectedValue, double acceptableDelta)
    {
        double actualDelta = expectedValue - actualValue;
        return Math.abs(actualDelta) <= acceptableDelta;
    }
}
