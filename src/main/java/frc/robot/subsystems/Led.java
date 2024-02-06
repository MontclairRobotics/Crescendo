package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.animation3.LedState;

public class Led extends SubsystemBase {
    private AddressableLED led = new AddressableLED(Constants.Ports.LED_PWM_PORT);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.BUFFER_SIZE);

    public Led() {
        led.setLength(Constants.LedConstants.BUFFER_SIZE);

        led.setData(ledBuffer);
        led.start();
    }

    public void fill(Color targetColor) {
        for (int i = 0; i < Constants.LedConstants.BUFFER_SIZE; i++) {
            ledBuffer.setLED(i, targetColor);
        }
    }

    public void setHSV(int index, int h, int s, int v) {
        ledBuffer.setHSV(index, h, s, v);
    }

    public Command runAnimation(LedState targetState) {
        return Commands.runOnce(targetState.getAnimation(), RobotContainer.led);
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
        led.start();
    }
}
