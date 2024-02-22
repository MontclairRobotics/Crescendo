package animation2;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.math.Math555;
import animation2.api.AnimationBase;
import animation2.api.LEDBuffer;

public class FlashAnimation extends AnimationBase
{
    private Color flashColor;

    public FlashAnimation(Color color)
    {
        flashColor = color;
    }

    @Override
    public void render()
    {
        Color color = flashColor;
        double time = getTimeElapsed() % 2;

        int state = Math555.repeatingCycle(time, 0, 2, 2, 200);
        color = Math555.lerp(Color.kBlack, flashColor, state / 200.0);

        LEDBuffer.fill(getBuffer(), color);
    }
    
}