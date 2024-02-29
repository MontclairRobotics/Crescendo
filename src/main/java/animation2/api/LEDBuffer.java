package animation2.api;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Various helpful methods for dealing with {@link AddressableLEDBuffer}s */
public class LEDBuffer {
  private LEDBuffer() {}

  /** Fills the given {@link AddressableLEDBuffer} with the given {@link Color} */
  public static void fill(AddressableLEDBuffer ledBuffer, Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
  }

  /** Copies the given LED index from the first buffer to the second */
  public static void copy(int i, AddressableLEDBuffer from, AddressableLEDBuffer to) {
    to.setLED(i, from.getLED(i));
  }

  /** Copies all LEDs from the first buffer to the second */
  public static void copy(AddressableLEDBuffer from, AddressableLEDBuffer to) {
    assert from.getLength() == to.getLength()
        : "Cannot copy from a buffer of size "
            + from.getLength()
            + " to a buffer of differing size "
            + to.getLength();

    for (int i = 0; i < from.getLength(); i++) {
      copy(i, from, to);
    }
  }

  /** Flip the contents of the given buffer 180 degrees. */
  public static void flip(AddressableLEDBuffer buf) {
    for (int i = 0; i < buf.getLength() / 2; i++) {
      Color temp = buf.getLED(i);
      buf.setLED(i, buf.getLED(buf.getLength() - i - 1));
      buf.setLED(buf.getLength() - i - 1, temp);
    }
  }

  /**
   * Mirror the contents of the given buffer around its center, copying the left half to the right
   * half but inverted.
   */
  public static void mirror(AddressableLEDBuffer buf) {
    for (int i = 0; i < buf.getLength() / 2; i++) {
      buf.setLED(i, buf.getLED(buf.getLength() - i - 1));
    }
  }
}
