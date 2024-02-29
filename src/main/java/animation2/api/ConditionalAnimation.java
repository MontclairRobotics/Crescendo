package animation2.api;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

/**
 * An animation which chooses from a set of other animations based on conditions provided to it with
 * {@code #addCase(BooleanSupplier, Animation)}, and outputs them to its own buffer.
 *
 * <p>This animation does not actually own its buffer, but rather returns the buffers of the
 * animations it chooses from, to save time costs incurred by copying from them.
 *
 * <p>The order in which cases are added to matter, and if, while running the animation associated
 * to some case added second, the first case condition becomes true, this animation will immediately
 * switch to using the result of the first case.
 *
 * <p>This animation does not update each of its options every frame, but rather updates them only
 * when they are the currently rendered option.
 */
public class ConditionalAnimation extends SimpleAnimationBase {
  /**
   * One case of this conditional animation. Contains a {@link BooleanSupplier} and an {@link
   * Animation}
   */
  public static class Case {
    public final BooleanSupplier condition;
    public final Animation animation;

    public Case(BooleanSupplier condition, Animation animation) {
      this.condition = condition;
      this.animation = animation;
    }

    public boolean getCondition() {
      return condition.getAsBoolean();
    }
  }

  public ConditionalAnimation(Animation baseCase) {
    this.baseCase = baseCase;
  }

  /**
   * Add a case to this conditional animation. Note that, as is explained in the documentation for
   * this animation class, the order in which you add cases is meaningful.
   */
  public ConditionalAnimation addCase(BooleanSupplier condition, Animation animation) {
    cases.add(new Case(condition, animation));
    return this;
  }

  final Animation baseCase;
  final ArrayList<Case> cases = new ArrayList<>();

  /** The current choice selected. -1 represents the base case. */
  int currentChoice;

  @Override
  public AddressableLEDBuffer getBuffer() {
    return getCaseAnimation(currentChoice).getBuffer();
  }

  @Override
  public void render() {
    currentChoice = -1;
    for (int i = 0; i < cases.size(); i++) {
      if (cases.get(i).getCondition()) {
        currentChoice = i;
        break;
      }
    }

    getCaseAnimation(currentChoice).render();

    // System.out.println(getBuffer().getLED(0).toHexString());
  }

  private Animation getCaseAnimation(int x) {
    if (x == -1) return baseCase;
    return cases.get(x).animation;
  }

  @Override
  public void start() {
    baseCase.start();
    for (Case caze : cases) {
      caze.animation.start();
    }
  }
}
