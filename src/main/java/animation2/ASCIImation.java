package animation2;

import animation2.api.AnimationBase;
import edu.wpi.first.wpilibj.util.Color;

public class ASCIImation extends AnimationBase {
  private final Color[] bits;

  public ASCIImation(String text, Color lo, Color hi, Color buf, Color wrd) {
    final int tlen9 = text.length() * 9;
    bits = new Color[tlen9 + 1];

    bits[0] = wrd;
    bits[tlen9] = wrd;

    int idx = 1;
    for (char c : text.toCharArray()) {
      int ci = c;
      for (int i = 0; i < 8; i++) {
        bits[idx++] = (ci & 1) == 0 ? lo : hi;
        ci >>= 1;
      }

      if (idx != tlen9) {
        bits[idx++] = buf;
      }
    }
  }

  @Override
  public void render() {
    final int offset = (int) (getTimeElapsed() / 0.1);

    for (int i = 0; i < getBuffer().getLength(); i++) {
      final int x = (i + offset) % getBuffer().getLength();
      final Color c = bits[x % bits.length];

      getBuffer().setLED(i, c);
    }
  }
}
