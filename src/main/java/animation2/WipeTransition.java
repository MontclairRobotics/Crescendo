package animation2;

import animation2.api.LEDBuffer;
import animation2.api.TransitionBase;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * An animation which wipes from one color to another (without any blurring).
 */
public class WipeTransition extends TransitionBase 
{
    @Override
    public void render()
    {
        super.render();
        
        final int wipeOffset = (int)(getBuffer().getLength() * getPercentComplete());

        LEDBuffer.copy(getOut().getBuffer(), getBuffer());

        for (int i = 0; i < wipeOffset; i++) 
        {
            LEDBuffer.copy(i, getIn().getBuffer(), getBuffer());
        }
    }
}
