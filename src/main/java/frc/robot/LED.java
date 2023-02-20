package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    private AddressableLED led;
    private AddressableLEDBuffer ledLength;

    public LED(int DigitalPort, int bufferLength) {
        led = new AddressableLED(DigitalPort); 
        ledLength = new AddressableLEDBuffer(bufferLength); 
        
        led.setLength(ledLength.getLength());

        led.setData(ledLength);
        led.start();
    }

    public void stop() {
        led.stop();
    }

    public void setRGB(int[] rgb) {
        for (int i = 0; i < ledLength.getLength(); i++) {
            ledLength.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
        
        led.setData(ledLength);
        led.start();
    }
}