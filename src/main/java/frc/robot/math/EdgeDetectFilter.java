package frc.robot.math;

/**
 * A filter, like those in wpilib, which detects rising and falling edges in a boolean signal.
 * Should only operate on one data stream, and should also be 'reset' when its data is, since it has
 * internal memory.
 */
public class EdgeDetectFilter {
  /** The type of edge to detect. */
  public static enum EdgeType {
    /** A rising edge; {@code false} -> {@code true} */
    RISING,
    /** A falling edge; {@code true} -> {@code false} */
    FALLING,
    /** Either a rising or falling edge; the value has changed */
    EITHER;
  }

  private boolean last;
  private boolean current;

  private final EdgeType detectionType;

  public EdgeDetectFilter(EdgeType type) {
    detectionType = type;
  }

  /** Reset the internal state of this filter. */
  public void reset() {
    last = false;
    current = true;
  }

  /**
   * Return if the signal has just experienced a rising edge; i.e. it went from {@code false} to
   * {@code true} in the last update.
   */
  public boolean isRising() {
    return current && !last;
  }

  /**
   * Return if the signal has just experienced a falling edge; i.e. it went from {@code true} to
   * {@code false} in the last update.
   */
  public boolean isFalling() {
    return !current && last;
  }

  /**
   * Return if the signal has just experienced an edge; i.e. it changed value in the last update.
   */
  public boolean isEdge() {
    return current ^ last;
  }

  /**
   * Update the state of this filter with new data, and return if the configured {@link
   * EdgeDetectFilter.EdgeType} has just occured. After calling this function, the return values of
   * {@link #isRising()}, {@link #isFalling()}, and {@link #isEdge()} will be updated with the new
   * data.
   */
  public boolean calculate(boolean value) {
    last = current;
    current = value;

    if (detectionType == EdgeType.RISING) return isRising();
    if (detectionType == EdgeType.FALLING) return isFalling();
    if (detectionType == EdgeType.EITHER) return isEdge();

    System.out.println("Error in EdgeDetectFilter.java");
    return false;
  }
}
