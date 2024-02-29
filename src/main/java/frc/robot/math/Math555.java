package frc.robot.math;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Arrays;

public class Math555 {
  private Math555() {}

  /**
   * Clamp the Value between the min and max
   *
   * @param v
   * @param min
   * @param max
   * @return
   */
  public static double clamp(double v, double min, double max) {
    if (v < min) return min;
    if (v > max) return max;

    return v;
  }

  /**
   * Clamp the Value between min and max
   *
   * @param v
   * @param min
   * @param max
   * @return
   */
  public static int clamp(int v, int min, int max) {
    if (v < min) return min;
    if (v > max) return max;

    return v;
  }

  /**
   * Clamp Target between the current minus maxDec and current plus maxDec
   *
   * @param target
   * @param current
   * @param maxAcc
   * @param maxDec
   * @return
   */
  public static double accClamp(double target, double current, double maxAcc, double maxDec) {
    if (Math.abs(current) > Math.abs(target)) {
      return Math555.clamp(target, current - maxDec, current + maxDec);
    } else {
      return Math555.clamp(target, current - maxAcc, current + maxAcc);
    }
  }

  /**
   * Returns <code>value</code>, <b>UNLESS</b>, its magnitude is below the minimum; in which case it
   * returns <code>min</code>
   *
   * @param value
   * @param min
   * @return
   */
  public static double atLeast(double value, double min) {
    return Math.abs(value) < min ? min * Math.signum(value) : value;
  }

  /**
   * Clamp the value between 0 and 1
   *
   * @param v
   * @return
   */
  public static double clamp01(double v) {
    return clamp(v, 0, 1);
  }

  /**
   * @param a minimum value
   * @param b maximum value
   * @param t fraction
   * @return a value between a and b, based on a fraction t
   */
  public static double lerp(double a, double b, double t) {
    t = clamp01(t);
    return t * (b - a) + a;
  }

  /**
   * @param a minimum value
   * @param b maximum value
   * @param t fraction
   * @return a value between a and b, based on a fraction t
   */
  public static int lerp(int a, int b, double t) {
    t = clamp01(t);
    return (int) (t * (b - a) + a);
  }

  /**
   * returns a fraction based on the value from a to b.
   *
   * <p>If the value is less than the minimum, it returns 0 If the value is greater than the
   * maximum, it returns 1
   *
   * @param v value
   * @param a mininum
   * @param b maximum
   * @return a fraction based on a value between a and b
   */
  public static double invlerp(double v, double a, double b) {
    return clamp01((v - a) / (b - a));
  }

  public static double invlerpNoClamp(double v, double a, double b) {
    return (v - a) / (b - a);
  }

  /**
   * returns a fraction based on the value from a to b.
   *
   * <p>If the value is less than the minimum, it returns 0 If the value is greater than the
   * maximum, it returns 1
   *
   * @param v value
   * @param a mininum
   * @param b maximum
   * @return a fraction based on a value between a and b
   */
  public static double invlerp(int v, int a, int b) {
    return clamp01((double) (v - a) / (b - a));
  }

  public static double invlerpNoClamp(int v, int a, int b) {
    return (double) (v - a) / (b - a);
  }

  /**
   * This method does a linear interpolation between 2 colors by lerping between each of the RGB
   * values. It uses this {@link Math555#lerp(int, int, double) Lerp Function}
   *
   * @param a Start Color
   * @param b End Color
   * @param t fraction
   * @return a Color between a and b, based on a fraction t
   */
  public static Color lerp(Color a, Color b, double t) {
    return new Color(lerp(a.red, b.red, t), lerp(a.green, b.green, t), lerp(a.blue, b.blue, t));
  }

  public static Color8Bit lerp(Color8Bit a, Color8Bit b, double t) {
    return new Color8Bit(lerp(a.red, b.red, t), lerp(a.green, b.green, t), lerp(a.blue, b.blue, t));
  }

  /**
   * Uses {@link Math555#invlerp(double, double, double) inverse lerp function}, we get the value
   * from [0, cycleLength-1]. It maps these values (0, 1, 2, ... cycleLength) cycles number of times
   *
   * @param value
   * @param minimum
   * @param maximum
   * @param cycles
   * @param cycleLength
   * @return
   */
  public static int repeatingCycle(
      double value, double minimum, double maximum, int cycles, int cycleLength) {
    return (int) (Math555.invlerpNoClamp(value, minimum, maximum) * cycles * cycleLength)
        % cycleLength;
  }

  public static double max(double... ds) {
    return Arrays.stream(ds).max().orElse(Double.NaN);
  }

  public static double min(double... ds) {
    return Arrays.stream(ds).min().orElse(Double.NaN);
  }

  public static int max(int... ds) {
    return Arrays.stream(ds).max().orElse(Integer.MAX_VALUE);
  }

  public static int min(int... ds) {
    return Arrays.stream(ds).min().orElse(Integer.MAX_VALUE);
  }

  public static double signpow(double v, double pow) {
    if (v > 0) return +Math.pow(+v, pow);
    else return -Math.pow(-v, pow);
  }

  /**
   * clamp value between -1 and 1
   *
   * @param v
   * @return
   */
  public static double clamp1(double v) {
    return clamp(v, -1, 1);
  }
}
