package animation2.api;

/**
 * Implements a basic transition which interpolates momentarily between two animations.
 *
 * <p>Note that implementers must call this class' {@link #render()} in order to keep the in and out
 * animations up-to-date.
 */
public abstract class TransitionBase extends TimedAnimationBase {
  private Animation out;
  private Animation in;

  public void setOut(Animation anim) {
    out = anim;
  }

  public void setIn(Animation anim) {
    in = anim;
  }

  protected Animation getOut() {
    return out;
  }

  protected Animation getIn() {
    return in;
  }

  @Override
  public void render() {
    out.render();
    in.render();
  }
}
