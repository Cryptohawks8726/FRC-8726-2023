package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    private AddressableLED m_led; // will contain the AddressableLED object
    private AddressableLEDBuffer m_ledBuffer; // will contain the AddressableLEDBuffer object

    public LED(int DigitalPort, int bufferLength) {
        // Takes the PWM port as the parameter
        m_led = new AddressableLED(DigitalPort); // default 9
        m_ledBuffer = new AddressableLEDBuffer(bufferLength); // default 60
        
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data

    }

    public void ledStart() {
        System.out.println("YES");
        m_led.start();
    }

    public void ledSetRGB(int R, int G, int B) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values inputted
            m_ledBuffer.setRGB(i, R, G, B);
        }
       
       m_led.setData(m_ledBuffer);
    }
}