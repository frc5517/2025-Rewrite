package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.AddressableLEDSubsystem.PatternConstants.*;
import static frc.robot.subsystems.AddressableLEDSubsystem.HardwareConstants.*;

public class AddressableLEDSubsystem extends SubsystemBase {

    public static final class PatternConstants {
        public static final LinearVelocity kRainbowScrollVelocity = MetersPerSecond.of(8);
        public static final Time kBlinkOnTime = Seconds.of(1);
        public static final Time kBlinkOffTime = Seconds.of(1);
    }
    public static final class HardwareConstants {
        public static final int kLedPort = 0; // PWM
        public static final int kLedLength = 320;
        public static final Distance kLedSpacing = Meter.of(1.0 / 60);
        public static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLedLength);
        public static final AddressableLEDBufferView underGlow = buffer.createView(0, 100);
        public static final AddressableLEDBufferView elevatorGlow = buffer.createView(101, 319);
    }

    private final AddressableLED led = new AddressableLED(kLedPort);
    private LEDPattern elevatorPattern = LEDPattern.solid(editColor(Color.kPurple));
    private LEDPattern underPattern = LEDPattern.solid(editColor(Color.kFirstBlue));

    public AddressableLEDSubsystem() {
        led.setLength(buffer.getLength());
        led.start();
    }

    private LEDPattern prevElevPattern = null;
    private LEDPattern prevUnderPattern = null;
    @Override
    public void periodic() {
        if (prevElevPattern != elevatorPattern) {
            elevatorPattern.applyTo(elevatorGlow);
            prevElevPattern = elevatorPattern;
        }
        if (prevUnderPattern != underPattern) {
            underPattern.applyTo(underGlow);
            prevUnderPattern = underPattern;
        }
        led.setData(buffer);
    }

    public void runLED(LEDViews ledView, LEDModes ledMode, Color color) {
        LEDPattern pattern = switch (ledMode) {
            case RAINBOW -> rainbow();
            case SOLID -> solid(color);
            case OFF -> off();
        };
        setPattern(ledView, pattern);
    }

    public void  runLED(LEDViews ledView, LEDModes ledMode) {
        runLED(ledView, ledMode, Color.kBlack);
    }

    public void setPattern(LEDViews ledView, LEDPattern pattern) {
        switch (ledView) {
            case ELEVATOR -> setPatternElevator(pattern);
            case UNDER -> setPatternUnder(pattern);
            case BOTH -> setPatternBoth(pattern);
        }
    }

    public void setPatternElevator(LEDPattern pattern) {
        elevatorPattern = pattern;
    }

    public void setPatternUnder(LEDPattern pattern) {
        underPattern = pattern;
    }

    public void setPatternBoth(LEDPattern pattern) {
        setPatternUnder(pattern);
        setPatternElevator(pattern);
    }

    private Color editColor(Color color) {
        double red = color.green;
        double green = color.red;
        double blue = color.blue;
        return new Color(red, green, blue);
    }

    public enum LEDViews {
        ELEVATOR,
        UNDER,
        BOTH
    }

    public enum LEDModes {
        RAINBOW,
        SOLID,
        OFF
    }

    ///
    /// LEDPatterns
    ///

    public LEDPattern rainbow() {
        return LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(kRainbowScrollVelocity, kLedSpacing);
    }

    public LEDPattern solid(Color color) {
        return LEDPattern.solid(editColor(color));
    }

    public LEDPattern continuousGradient(Color color) {
        return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, editColor(color));
    }

    public LEDPattern discontinuousGradient(Color color) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, editColor(color));
    }

    public LEDPattern blink(Color color) {
        return LEDPattern.solid(editColor(color)).blink(kBlinkOnTime, kBlinkOffTime);
    }

    public LEDPattern off() {
        return LEDPattern.kOff;
    }
}
