package frc.robot.utils.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.util.Timer;
import java.util.TimerTask;

public class AddressableLEDController {
    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;
    private AddressableLEDPattern addressableLEDPattern;

    private Timer timer;
    private TimerTask task;
    private int animationDelay;

    public AddressableLEDController(int pwmPort, int length) {
        addressableLED = new AddressableLED(pwmPort);
        addressableLEDBuffer = new AddressableLEDBuffer(length);
        addressableLED.setLength(length);
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    public void setAddressableLEDPattern(AddressableLEDPattern newPattern) {
        if (addressableLEDPattern != newPattern) {
            addressableLEDPattern = newPattern;
            if (task != null) {
                task.cancel();
                task = null;
            }
            if (newPattern.isAnimated()) {
                task = new TimerTask() {
                    @Override
                    public void run() {
                        update();
                    }
                };
                timer.scheduleAtFixedRate(task, 20,  animationDelay);
            }
        }
        update();
    }

    public void update() {
        addressableLEDPattern.setLEDs(addressableLEDBuffer);
        addressableLED.setData(addressableLEDBuffer);
    }
}
