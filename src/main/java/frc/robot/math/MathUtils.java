package frc.robot.math;

public class MathUtils {

    /**
     * Clamps a value
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
}
