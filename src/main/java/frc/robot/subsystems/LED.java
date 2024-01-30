package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import animation2.AnimationStack;
import animation2.CelebrationAnimation;
import animation2.RaceAnimation;
import animation2.RainbowAnimation;
import animation2.api.Animation;
import animation2.api.LEDBuffer;
import animation2.api.TransitionBase;
//import org.team555.util.frc.commandrobot.ManagerBase;

public class LED extends SubsystemBase 
{
    private AddressableLED led = new AddressableLED(Constants.LEDConstants.LED_PWM_PORT);
    private AnimationStack stack;

    public static final double TRANSITION_LENGTH = 0.4;
    public static final int LED_COUNT = 95; //Completely arbitrary


    public LED(Animation defaultAnimation, TransitionBase transition)
    {
        stack = new AnimationStack(defaultAnimation, transition, TRANSITION_LENGTH);
        stack.start();

        led.setLength(LED_COUNT);
        led.start();
    }
   
    @Override
    public void periodic()
    {
        stack.render();
        led.setData(stack.getBuffer());
    }

    /**
     * Add a command to the stack, interrupting the previous command
     */
    public void add(Animation animation) 
    {
        stack.push(animation);
    }

    public void celebrate()
    {
        add(new CelebrationAnimation().length(1));
    }

    /**
     * Cancel the top command and return to the previous
     * @return The cancelled command
     */
    public void pop() 
    {
        stack.pop();
    }

    /**
     * Replace the current command with the given command
     * @return The cancelled command
     */
    public void replace(Animation animation)
    {
        stack.pop();
        stack.push(animation);
    }
}