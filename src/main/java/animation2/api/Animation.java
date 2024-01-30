package animation2.api;

import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * The base interface of all animations.
 * Defines necessary methods but also some helper methods.
 */
public interface Animation 
{
    /**
     * Get the {@link AddressableLEDBuffer} which this animation is rendered in.
     * Does not update until {@link #render()} is called.
     */
    AddressableLEDBuffer getBuffer();

    /**
     * Initialize / reset the given animation.
     */
    void start();
    /**
     * Render the given animation into its internal buffer.
     */
    void render();

    /**
     * Check if this animation is completed. Note that an animation being completed
     * does not imply that its {@link #render()} cannot be called anymore, but rather
     * that it should "fade out".
     */
    boolean isFinished();

    /**
     * Get the underlying animation object associated to this animation. Useful to 
     * inspect the properties of an animation which has been modified by methods like
     * {@link #timeout(double)}.
     */
    default Animation getUnmodified() {return this;}

    /**
     * Return an animation wrapping this one which will end after {@code time} seconds
     */
    default Animation timeout(double time)
    {
        final Animation original = this;
        return new Animation() 
        {
            Timer t = new Timer();

            @Override
            public void start() 
            {
                original.start();
                t.reset();
                t.start();
            }

            @Override
            public AddressableLEDBuffer getBuffer() {return original.getBuffer();}

            @Override
            public void render() {original.render();}

            @Override
            public boolean isFinished() 
            {
                return original.isFinished() || t.hasElapsed(time);
            }

            @Override
            public Animation getUnmodified() 
            {
                return original;
            }
        };
    }
    /**
     * Return an animation which wraps this one, behaving identically except calling {@code fn}
     * after this animation's render method.
     */
    default Animation afterRender(Consumer<Animation> fn)
    {
        final Animation original = this;
        return new Animation() 
        {
            @Override
            public void start() {original.start();}

            @Override
            public boolean isFinished() {return original.isFinished();}

            @Override
            public AddressableLEDBuffer getBuffer() {return original.getBuffer();}

            @Override
            public void render() 
            {
                original.render();
                fn.accept(original);
            }

            @Override
            public Animation getUnmodified() 
            {
                return original;
            }
        };
    }
    /**
     * Creates a new animation wrapping this one which flips the render 180 degrees.
     */
    default Animation flip()
    {
        return afterRender(x -> LEDBuffer.flip(x.getBuffer()));
    }
    /**
     * Creates a new animation wrapping this one which mirrors the render from its center, 
     * taking the left side and copying it to the right.
     */
    default Animation mirror()
    {
        return afterRender(x -> LEDBuffer.mirror(x.getBuffer()));
    }
    /**
     * Creates a new animation wrapping this one which randomly chooses to 
     * flip and/or mirror itself each time it is started.
     */
    default Animation randomized()
    {
        final Animation original = this;
        return new Animation() 
        {
            boolean flipped;
            boolean mirrored;

            @Override
            public AddressableLEDBuffer getBuffer() {return original.getBuffer();}

            @Override
            public void render() 
            {
                original.render();

                if(flipped) LEDBuffer.flip(getBuffer());
                if(mirrored) LEDBuffer.mirror(getBuffer());
            }

            @Override
            public boolean isFinished() 
            {
                return original.isFinished();
            }

            @Override
            public void start() 
            {
                original.start();

                Random rand = new Random(System.currentTimeMillis());
                flipped = rand.nextBoolean();
                mirrored = rand.nextBoolean();
            }
            
            @Override
            public Animation getUnmodified() 
            {
                return original;
            }
        };
    }
    /**
     * Returns a new animation wrapping this one which copies its buffer to a fresh
     * buffer on each render call. For this reason, it is valid to write, for instance
     * {@code led.setData(myAnimation.buffer())} if {@code myAnimation = myAnimationInternal.copied()}.
     */
    default Animation copied()
    {
        final Animation original = this;
        return new AnimationBase() 
        {
            @Override
            public void render() 
            {
                original.render();
                LEDBuffer.copy(original.getBuffer(), this.getBuffer());
            }

            @Override
            public Animation getUnmodified() 
            {
                return original;
            }
        };
    }
}
