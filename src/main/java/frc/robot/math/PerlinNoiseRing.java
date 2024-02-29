package frc.robot.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Random;

public class PerlinNoiseRing {
  public PerlinNoiseRing(int step, int size) {
    // Get a random movement direction
    movementDir = new Rotation2d(new Random().nextDouble() * Math.PI);

    this.size = size;
    this.step = step;
  }

  public PerlinNoiseRing() {
    this(40, 70);
  }

  // Constants //
  // The amount of movement in one second
  public double step;
  // The size of the circle in Perlin space
  public double size;

  private Rotation2d movementDir;
  private PerlinNoise noise = new PerlinNoise();

  /**
   * The way that this algorithm works is not self-explanatory, so I will explain it here:
   *
   * <p>In order to simulate flames, we can use randomness to generate a "heat" value, which can
   * then be translated into a color and a brightness.
   *
   * <p>This "heat" value needs to be random yet smooth, and changeable smoothly over time. For this
   * reason, Perlin noise is perfect, since it - is deterministic for a given seed - is "smooth" -
   * is "random"
   *
   * <p>In order to get a pattern which "wraps" around the edges of the LED strip though, we will
   * need to somehow make our input coordinates to the perlin noise generator similar for opposing
   * ends of the strip.
   *
   * <p>We can do this by treating the current led index as an "angle" around a circle of leds,
   * where 0 -> 0 radians and N -> 2pi radians if the LED strip is N LEDs long.
   *
   * <p>We can therefore sample each LED's "heat" value using the following pseudo-code: heat =
   * noise(center + vec2D.fromMagnitude(SIZE).rotate(ledIndex / ledLength))
   *
   * <p>Implementing this is a matter of translating these operations into both wpilib and integer
   * or double math.
   *
   * <p>In order to make our flames evolve over time though, we must move our circle around the
   * perlin noise space as time advances.
   *
   * <p>The simplest way to do this--which fortunately does not rely on persistent, mutable fields--
   * is to store a "movement direction" vector and multiply it by the time elapsed since the
   * animation's start (timer.get()) in order to get the center position.
   *
   * <p>This movement direction can be initialized alongside the animation using java's Random
   * class.
   *
   * <p>Finally, to go from our "heat" value to an actual color value, we need to linearly
   * interpolate between a "cool" flame color and a "hot" flame color as well as a dim brightness
   * and high brightness, giving us the components of an HSV color (assuming saturation is always
   * 255).
   *
   * <p>Because of the nature of linear interpolation, this means that we must re-map our generated
   * "heat" value from [-1, 1], the range provided by PerlinNoise, to [0, 1]. This can be done with
   * x = x / 2.0 + 0.5
   *
   * <p>In order to make our flames appear more distinct, we raise this remapped heat value to some
   * power, effectively increasing the threshold of heat values which will produce visibly hot
   * pixels.
   *
   * <p>We can then pipe the results of the two lerps for hue and val into ledBuffer.setHSV to
   * render our current animation pixel, for every pixel.
   */
  public double get(double time, int i, int bufferLen) {
    Translation2d center = new Translation2d(step * time, movementDir);

    // Get the "normalized" buffer index (between 0 and 1)
    double inorm = (1.0 * i) / bufferLen;

    // Get the angle offset of this pixel (between 0 and 2pi)
    Rotation2d angle = new Rotation2d(2.0 * Math.PI * inorm);

    // Get the vector offset of this pixel and calculate its position by adding this offset
    // to the center of the circle
    Translation2d offset = new Translation2d(size, angle);
    Translation2d position = center.plus(offset);

    // Get the heat value for the position found above
    double heat = noise.noise(position.getX(), position.getY());

    // Remap this value
    double heatRemap = heat / 2.0 + 0.5;

    return heatRemap;

    // Perform the linear interpolations needed to get the hue and value,
    // using (0 -> 10) as the hue range and (0 -> 255) for the value range.
    // To change the color of the flames, modify these values.
    // int hue = Math555.lerp(0, 20,  Math.pow(heatRemap, 2.0));
    // int val = Math555.lerp(0, 255, Math.pow(heatRemap, 4.0));

  }
}
