package animation2;

import animation2.api.Animation;
import animation2.api.LEDBuffer;
import animation2.api.SimpleAnimationBase;
import animation2.api.TimedAnimationBase;
import animation2.api.TransitionBase;
//import frc.robot.util.Array555;
import frc.robot.math.EdgeDetectFilter;
import frc.robot.math.EdgeDetectFilter.EdgeType;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Provides a reel of animations. Used for the disabled demo.
 */
public class AnimationReel extends SimpleAnimationBase
{
    private final Animation[] animations;
    private final double reeltime;
    private final double transtime;

    private final TransitionBase transition;

    private boolean isTransitioning;
    private EdgeDetectFilter startTransitioning = new EdgeDetectFilter(EdgeType.RISING);

    private int index;

    public AnimationReel(double reeltime, double transtime, TransitionBase transition, Animation... animations)
    {
        assert animations.length > 1 : "Must have at least two animations to create a reel!";

        this.animations = animations;
        this.reeltime = reeltime;
        this.transtime  = transtime;
        this.transition = transition;

        transition.setLength(transtime);

        for(Animation animation : animations)
        {
            Animation unmodified = animation.getUnmodified();
            
            if(unmodified instanceof TimedAnimationBase)
            {
                ((TimedAnimationBase)unmodified).setLength(reeltime);
            }
        }
    }

    @Override
    public AddressableLEDBuffer getBuffer() 
    {
        if(isTransitioning) return transition.getBuffer();

        return animations[index].getBuffer();
    }

    @Override
    public void start()
    {
        super.start();

        startTransitioning.calculate(false);
        index = 0;

        animations[0].start();
    }

    @Override
    public void render() 
    {
        // Calculate new values
        double timeInReel = getTimeElapsed() % (reeltime + transtime);
        index = (int)(getTimeElapsed() / (reeltime + transtime)) % animations.length;

        isTransitioning = timeInReel >= reeltime;
        boolean isJustTrans = startTransitioning.calculate(isTransitioning);

        int nextIndex = (index + 1) % animations.length;
        
        // Set up transition if necessary
        if(isJustTrans) 
        {
            transition.start();

            animations[nextIndex].start();

            transition.setOut(animations[index]);
            transition.setIn(animations[nextIndex]);
        }

        // Run transition
        if(isTransitioning)
        {
            transition.render();
        }
        // Run animation
        else 
        {
            animations[index].render();
        }
    }
}
