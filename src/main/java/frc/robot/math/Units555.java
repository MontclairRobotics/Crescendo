package frc.robot.math;

public class Units555
{
    /**
     * Convert from miles per hour to meters per second
     * @param value Velocity <b>(miles per hour))</b>
     * @return Velocity <b>(meters per second)</b>
     */
    public static double miphToMps(double value)
    {
        return value * 0.44704;
    }
    /**
     * Convert from meters per second to miles per hour
     * @param value Velocity <b>(meters per second)</b>
     * @return Velocity <b>(miles per hour))</b>
     */
    public static double mpsToMiph(double value)
    {
        return value / 0.44704;
    }

    /**
     * Convert from miles per hour per second to meters per second squared
     * @param value Acceleration <b>(miles per hour per sec)</b>
     * @return Acceleration <b>(meters per sec^2)<b>
     */
    public static double miphpsToMpsps(double value)
    {
        return value * 0.44704;
    }
    /**
     * Convert from meters per second squared to miles per hour per second
     * @param value Acceleration <b>(meters per sec^2)<b>
     * @return Acceleration <b>(miles per hour per sec)</b>
     */
    public static double mpspsToMiphps(double value)
    {
        return value / 0.44704;
    }

    /**
     * Convert from Pounds Feet Squared to Kilograms Meters Squared
     * @param value Moment of Inertia (pounds feet squared)
     * @return Moment of Inertia (Kilograms Meters Squared)
     */
    public static double lbFt2ToKgM2(double value)
    {
        return value * 0.0421401101;
    }
    /**
     * Convert from Kilograms Meters Squared to Pounds Feet Squared
     * @param value Moment of Inertia (Kilograms Meters Squared)
     * @return Moment of Inertia (pounds feet squared)
     */
    public static double kgM2ToLbFt2(double value)
    {
        return value / 0.0421401101;
    }
}
