package animation2;

import animation2.api.AnimationBase;
import java.util.Random;

public class DeathAnimation extends AnimationBase {
  @Override
  public void render() {
    for (int i = 0; i < getBuffer().getLength(); i++) {
      Random random = new Random();
      getBuffer().setHSV(i, random.nextInt(256), random.nextInt(256), random.nextInt(256));
    }
  }
}
