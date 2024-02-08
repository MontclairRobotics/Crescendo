package animation2;

import edu.wpi.first.wpilibj.util.Color;
import animation2.api.AnimationBase;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;


public class DuckAnimation extends AnimationBase {
String msg = "-.. ..- -.-. -.-  -.-- --- ..-  ..-. -.-- .-.. .- -.   ";
char[] msgArray = msg.toCharArray();
//one dot = dot, three dots = dash, one blank dot = space between dots & dashes, two blank dots = space between letters, red dot = space between words
private final Color background = Color.kBlack;
    public int offset = 0;
    public int index = 0;

    public static final double SHIFT_SPEED = 0.1;

    public void dot(int index) {
        getBuffer().setLED(index, Color.kLavender);
    }
    public void dash(int index) {
        getBuffer().setLED(index, Color.kCoral);
        getBuffer().setLED(index+1, Color.kCoral);
        getBuffer().setLED(index+2, Color.kCoral);
    }

    public void blank(int index) {
        getBuffer().setLED(index, Color.kBlack);
    }


    public void render() {
        offset = (int)(getTimeElapsed() / SHIFT_SPEED);
        index = 0;
        //wrap around: if greater than ledLength, index = ledLength - index?
        for(int i = 0; i < getBuffer().getLength(); i++) {
            if (msgArray[i] == '.') {
                dot(offset + index);
                index += 1;
            } else if (msgArray[i] == '-') {
                dash(offset + index);
                index += 3;
            } else if (msgArray[i] == ' ') {
                blank(offset + index);
                index += 1;
            }
        }  
    }
}
