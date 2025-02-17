package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;

    public void ConfigureLEDs() {
        addressableLED = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(60);
        addressableLED.setLength(ledBuffer.getLength());
        addressableLED.setData(ledBuffer);
        addressableLED.start();
    }

    public void SetLEDBuffer(int r, int g, int b){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
          }
          addressableLED.setData(ledBuffer);
    }
    
    public static float pingPong(double time, float maxValue) {
        return (float) (maxValue - Math.abs((time % (maxValue * 2)) - maxValue));
    }

    public void FlashLed(){
        double ledStrength = Math.round(pingPong(Timer.getFPGATimestamp() * 8, 1)) * 255;
        SetLEDBuffer(0, (int)ledStrength, 0);
    }
}
