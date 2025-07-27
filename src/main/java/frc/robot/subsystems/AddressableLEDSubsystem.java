package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.AddressableLEDSubsystem.ControlConstants.*;
import static frc.robot.subsystems.AddressableLEDSubsystem.HardwareConstants.*;

public class AddressableLEDSubsystem extends SubsystemBase {

    public static final class ControlConstants {
        public static final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    }
    public static final class HardwareConstants {
        public static final int kLedPort = 0; // PWM
        public static final int kLedLength = 200;
        public static final Distance kLedSpacing = Meter.of(1.0 / 60);
        public static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(320);
        public static final AddressableLEDBufferView underGlow = buffer.createView(0, 100);
        public static final AddressableLEDBufferView elevatorGlow = buffer.createView(101, 319);
    }

    private final AddressableLED led = new AddressableLED(HardwareConstants.kLedPort);
    private LEDPattern elevatorPattern = LEDPattern.solid(editColor(Color.kPurple));
    private LEDPattern underPattern = LEDPattern.solid(editColor(Color.kFirstBlue));

    public AddressableLEDSubsystem() {

        led.setLength(HardwareConstants.buffer.getLength());

        led.start();

        scrollingRainbow(5);
    }

    @Override
    public void periodic() {
        elevatorPattern.applyTo(elevatorGlow);
        underPattern.applyTo(underGlow);
        led.setData(HardwareConstants.buffer);

        //scrollingRainbow(5);
    }

    public Color editColor(Color color) {
        double red = color.green;
        double green = color.red;
        double blue = color.blue;
        return new Color(red, green, blue);
    }

    public void setProgressMask(Color progressColor, Color defaultColor, double progress, double maxProgress) {
        elevatorPattern = LEDPattern.progressMaskLayer(() -> progress / maxProgress);
    }

    public void scrollingRainbow(double scrollMetersPerSecond) {
        elevatorPattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(scrollMetersPerSecond), kLedSpacing);
        underPattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(scrollMetersPerSecond), kLedSpacing);
    }

    public Command scrollingRainbowCommand(boolean isUnder, boolean isElevator, double scrollMetersPerSecond) {
        return run(() -> {
            if (isUnder) {
                underPattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(scrollMetersPerSecond), kLedSpacing);
            }
            if (isElevator) {
                elevatorPattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(scrollMetersPerSecond), kLedSpacing);
            }
        });
    }

    public void setPatternElevator(LEDPattern pattern) {
        elevatorPattern = pattern;
    }

    public void setPatternUnder(LEDPattern pattern) {
        underPattern = pattern;
    }

    public Command runPatternElevator(LEDPattern pattern) {
        return runEnd(() -> setPatternElevator(pattern),
                () -> {
                    scrollingRainbow(5);
                });
    }

    public Command runPatternUnder(LEDPattern pattern) {
        return runEnd(() -> setPatternUnder(pattern),
                () -> {
                    scrollingRainbow(5);
                });
    }

    public Command runPatternBoth(LEDPattern underPattern, LEDPattern elevatorPattern) {
        return runEnd(() -> {
            setPatternUnder(underPattern);
            setPatternElevator(elevatorPattern);
        }, () -> scrollingRainbow(5));

    }

    // Will add as I decide how exactly to set up the LEDs

}
