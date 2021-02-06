package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.Executors;

// FRC Imports
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * @author Mark Ebert
 */
public class LightingController {

    private final AddressableLED ledController;
    private final AddressableLEDBuffer ledBuffer;

    private volatile Pattern selectedPattern;
    private volatile Color primaryColor, secondaryColor;
    private volatile boolean patternChanged;
    private volatile double delay;

    public enum Pattern {
        Color_From_Center, Color_From_Edges, Color_From_Start, Color_From_End, Snake_From_Center, Snake_From_Edges,
        Snake_From_Start, Snake_From_End, Running_Lights_From_Center, Running_Lights_From_Edges,
        Running_Lights_From_Start, Running_Lights_From_End, Static_Color, Off
    }

    public LightingController(final int PWMChannel, final int length) {
        ledController = new AddressableLED(PWMChannel);
        ledController.setLength(length);
        ledBuffer = new AddressableLEDBuffer(length);
        ledController.setData(ledBuffer);
        ledController.start();

        selectedPattern = Pattern.Color_From_Center;
        primaryColor = Color.kGreen;
        secondaryColor = Color.kGreen;
        patternChanged = false;
        delay = .05;

        Executors.newSingleThreadExecutor().execute(() -> {
            while (true) {
                try {
                    switch (selectedPattern) {
                    case Color_From_Center:
                        colorFromCenter(primaryColor);
                        colorFromCenter(secondaryColor);
                        break;
                    case Color_From_Edges:
                        colorFromEdges(primaryColor);
                        colorFromEdges(secondaryColor);
                        break;
                    case Color_From_Start:
                        colorFromStart(primaryColor);
                        colorFromStart(secondaryColor);
                        break;
                    case Color_From_End:
                        colorFromEnd(primaryColor);
                        colorFromEnd(secondaryColor);
                        break;
                    case Snake_From_Center:
                        snakeFromCenter(primaryColor, secondaryColor);
                        snakeFromCenter(secondaryColor, primaryColor);
                        break;
                    case Snake_From_Edges:
                        snakeFromEdges(primaryColor, secondaryColor);
                        snakeFromEdges(secondaryColor, primaryColor);
                        break;
                    case Snake_From_Start:
                        snakeFromStart(primaryColor, secondaryColor);
                        snakeFromStart(secondaryColor, primaryColor);
                        break;
                    case Snake_From_End:
                        snakeFromEnd(primaryColor, secondaryColor);
                        snakeFromEnd(secondaryColor, primaryColor);
                        break;
                    case Running_Lights_From_Center:
                        runningLightsFromCenter(primaryColor);
                        break;
                    case Running_Lights_From_Edges:
                        runningLightsFromEdges(primaryColor);
                        break;
                    case Running_Lights_From_Start:
                        runningLightsFromStart(primaryColor);
                        break;
                    case Running_Lights_From_End:
                        runningLightsFromEnd(primaryColor);
                        break;
                    case Static_Color:
                        staticColor(primaryColor);
                        break;
                    case Off:
                    default:
                        // Set strip to off
                        staticColor(Color.kBlack);
                        break;
                    }
                    patternChanged = false;
                    Timer.delay(delay);
                } catch (Exception e) {
                    System.err.println(e.getMessage());
                }
            }
        });
    }

    public void setPattern(final Pattern pattern, final Color primaryColor, final Color secondaryColor) {
        if (selectedPattern != pattern) {
            selectedPattern = pattern;
            patternChanged = true;
        }
        if (this.primaryColor != primaryColor) {
            this.primaryColor = primaryColor;
            patternChanged = true;
        }
        if (this.secondaryColor != secondaryColor) {
            this.secondaryColor = secondaryColor;
            patternChanged = true;
        }
    }

    public void setPattern(final Pattern pattern) {
        if (selectedPattern != pattern) {
            selectedPattern = pattern;
            patternChanged = true;
        }
    }

    public void setDelay(final double delay) {
        if (delay > .25) {
            this.delay = .25;
        } else if (delay < .01) {
            this.delay = .01;
        } else {
            this.delay = delay;
        }
    }

    private void colorFromCenter(final Color color) {
        if (ledBuffer.getLength() % 2 == 1) {
            final int centerIndex = (ledBuffer.getLength() / 2);
            ledBuffer.setLED(centerIndex, color);
            ledController.setData(ledBuffer);
            Timer.delay(delay);

            for (int i = 1; i <= ledBuffer.getLength() / 2; i++) {
                if (patternChanged) {
                    return;
                }
                ledBuffer.setLED(centerIndex - i, color);
                ledBuffer.setLED(centerIndex + i, color);
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }
        } else {
            final int centerIndexOne = (ledBuffer.getLength() / 2) - 1;
            final int centerIndexTwo = (ledBuffer.getLength() / 2);
            ledBuffer.setLED(centerIndexOne, color);
            ledBuffer.setLED(centerIndexTwo, color);
            ledController.setData(ledBuffer);
            Timer.delay(delay);

            for (int i = 1; i < ledBuffer.getLength() / 2; i++) {
                if (patternChanged) {
                    return;
                }
                ledBuffer.setLED(centerIndexOne - i, color);
                ledBuffer.setLED(centerIndexTwo + i, color);
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }
        }
    }

    private void colorFromEdges(final Color color) {
        if (ledBuffer.getLength() % 2 == 1) {
            for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
                if (patternChanged) {
                    return;
                }
                ledBuffer.setLED(i, color);
                ledBuffer.setLED((ledBuffer.getLength() - 1) - i, color);
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }

            final int centerIndex = (ledBuffer.getLength() / 2);
            ledBuffer.setLED(centerIndex, color);
            ledController.setData(ledBuffer);
            Timer.delay(delay);
        } else {
            for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
                if (patternChanged) {
                    return;
                }
                ledBuffer.setLED(i, color);
                ledBuffer.setLED((ledBuffer.getLength() - 1) - i, color);
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }
        }
    }

    private void colorFromStart(final Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (patternChanged) {
                return;
            }
            ledBuffer.setLED(i, color);
            ledController.setData(ledBuffer);
            Timer.delay(delay);
        }
    }

    private void colorFromEnd(final Color color) {
        for (int i = ledBuffer.getLength() - 1; i >= 0; i--) {
            if (patternChanged) {
                return;
            }
            ledBuffer.setLED(i, color);
            ledController.setData(ledBuffer);
            Timer.delay(delay);
        }
    }

    private void snakeFromCenter(final Color primaryColor, final Color secondaryColor) {
        boolean alternate = true;
        if (ledBuffer.getLength() % 2 == 1) {
            final int commonMultiple = findClosestMultiple(ledBuffer.getLength() / 2, .1f);

            final int centerIndex = (ledBuffer.getLength() / 2);
            ledBuffer.setLED(centerIndex, primaryColor);
            ledController.setData(ledBuffer);
            Timer.delay(delay);

            for (int i = 1; i <= ledBuffer.getLength() / 2; i += commonMultiple) {
                if (patternChanged) {
                    return;
                }
                for (int x = i; x < i + commonMultiple; x++) {
                    if (alternate) {
                        ledBuffer.setLED(centerIndex - x, secondaryColor);
                        ledBuffer.setLED(centerIndex + x, secondaryColor);
                    } else {
                        ledBuffer.setLED(centerIndex - x, primaryColor);
                        ledBuffer.setLED(centerIndex + x, primaryColor);
                    }
                }
                ledController.setData(ledBuffer);
                alternate = !alternate;
                Timer.delay(delay);
            }
        } else {
            final int commonMultiple = findClosestMultiple((ledBuffer.getLength() / 2) - 1, .1f);

            final int centerIndexOne = (ledBuffer.getLength() / 2) - 1;
            final int centerIndexTwo = (ledBuffer.getLength() / 2);
            ledBuffer.setLED(centerIndexOne, primaryColor);
            ledBuffer.setLED(centerIndexTwo, primaryColor);
            ledController.setData(ledBuffer);
            Timer.delay(delay);

            for (int i = 1; i < ledBuffer.getLength() / 2; i += commonMultiple) {
                if (patternChanged) {
                    return;
                }
                for (int x = i; x < i + commonMultiple; x++) {
                    if (alternate) {
                        ledBuffer.setLED(centerIndexOne - x, secondaryColor);
                        ledBuffer.setLED(centerIndexTwo + x, secondaryColor);
                    } else {
                        ledBuffer.setLED(centerIndexOne - x, primaryColor);
                        ledBuffer.setLED(centerIndexTwo + x, primaryColor);
                    }
                }
                ledController.setData(ledBuffer);
                alternate = !alternate;
                Timer.delay(delay);
            }
        }
    }

    private void snakeFromEdges(final Color primaryColor, final Color secondaryColor) {
        boolean alternate = false;
        if (ledBuffer.getLength() % 2 == 1) {
            final int commonMultiple = findClosestMultiple(ledBuffer.getLength() / 2, .1f);

            for (int i = 0; i < ledBuffer.getLength() / 2; i += commonMultiple) {
                if (patternChanged) {
                    return;
                }
                for (int x = i; x < i + commonMultiple; x++) {
                    if (alternate) {
                        ledBuffer.setLED(x, secondaryColor);
                        ledBuffer.setLED((ledBuffer.getLength() - 1) - x, secondaryColor);
                    } else {
                        ledBuffer.setLED(x, primaryColor);
                        ledBuffer.setLED((ledBuffer.getLength() - 1) - x, primaryColor);
                    }
                }
                ledController.setData(ledBuffer);
                alternate = !alternate;
                Timer.delay(delay);
            }

            final int centerIndex = (ledBuffer.getLength() / 2);
            if (alternate) {
                ledBuffer.setLED(centerIndex, secondaryColor);
            } else {
                ledBuffer.setLED(centerIndex, primaryColor);
            }
            ledController.setData(ledBuffer);
            Timer.delay(delay);
        } else {
            final int commonMultiple = findClosestMultiple((ledBuffer.getLength() / 2) - 1, .1f);

            for (int i = 0; i < ledBuffer.getLength() / 2; i += commonMultiple) {
                if (patternChanged) {
                    return;
                }
                for (int x = i; x < i + commonMultiple; x++) {
                    if (alternate) {
                        ledBuffer.setLED(x, secondaryColor);
                        ledBuffer.setLED((ledBuffer.getLength() - 1) - x, secondaryColor);
                    } else {
                        ledBuffer.setLED(x, primaryColor);
                        ledBuffer.setLED((ledBuffer.getLength() - 1) - x, primaryColor);
                    }
                }
                ledController.setData(ledBuffer);
                alternate = !alternate;
                Timer.delay(delay);
            }
        }
    }

    private void snakeFromStart(final Color primaryColor, final Color secondaryColor) {
        boolean alternate = false;
        final int commonMultiple = findClosestMultiple(ledBuffer.getLength(), .1f);
        for (int i = 0; i < ledBuffer.getLength(); i += commonMultiple) {
            if (patternChanged) {
                return;
            }
            for (int x = i; x < i + commonMultiple; x++) {
                if (alternate) {
                    ledBuffer.setLED(x, secondaryColor);
                } else {
                    ledBuffer.setLED(x, primaryColor);
                }
            }
            ledController.setData(ledBuffer);
            alternate = !alternate;
            Timer.delay(delay);
        }
    }

    private void snakeFromEnd(final Color primaryColor, final Color secondaryColor) {
        boolean alternate = false;
        final int commonMultiple = findClosestMultiple(ledBuffer.getLength(), .1f);
        for (int i = ledBuffer.getLength() - 1; i >= 0; i -= commonMultiple) {
            if (patternChanged) {
                return;
            }
            for (int x = i; x > i - commonMultiple; x--) {
                if (alternate) {
                    ledBuffer.setLED(x, secondaryColor);
                } else {
                    ledBuffer.setLED(x, primaryColor);
                }
            }
            ledController.setData(ledBuffer);
            alternate = !alternate;
            Timer.delay(delay);
        }
    }

    private void runningLightsFromCenter(final Color color) {
        if (ledBuffer.getLength() % 2 == 1) {
            final int centerIndex = (ledBuffer.getLength() / 2);
            int r = (int) (((Math.sin(centerIndex) * 127 + 128) / 255) * (color.red * 255));
            int g = (int) (((Math.sin(centerIndex) * 127 + 128) / 255) * (color.green * 255));
            int b = (int) (((Math.sin(centerIndex) * 127 + 128) / 255) * (color.blue * 255));
            ledBuffer.setRGB(centerIndex, r, g, b);
            ledController.setData(ledBuffer);
            Timer.delay(delay);

            int position = 0;
            for (int j = 0; j < ledBuffer.getLength(); j++) {
                if (patternChanged) {
                    return;
                }
                position++; // = 0; //Position + Rate;
                for (int i = 1; i < ledBuffer.getLength() / 2; i++) {
                    // sine wave, 3 offset waves make a rainbow!
                    // float level = sin(i+Position) * 127 + 128;
                    // setPixel(i,level,0,0);
                    // float level = sin(i+Position) * 127 + 128;
                    r = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.red * 255));
                    g = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.green * 255));
                    b = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.blue * 255));
                    ledBuffer.setRGB(centerIndex - i, r, g, b);
                    ledBuffer.setRGB(centerIndex + i, r, g, b);
                }
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }
        } else {
            final int centerIndexOne = (ledBuffer.getLength() / 2) - 1;
            final int centerIndexTwo = (ledBuffer.getLength() / 2);

            int position = 0;
            for (int j = 0; j < ledBuffer.getLength(); j++) {
                if (patternChanged) {
                    return;
                }
                position++; // = 0; //Position + Rate;
                for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
                    // sine wave, 3 offset waves make a rainbow!
                    // float level = sin(i+Position) * 127 + 128;
                    // setPixel(i,level,0,0);
                    // float level = sin(i+Position) * 127 + 128;
                    int r = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.red * 255));
                    int g = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.green * 255));
                    int b = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.blue * 255));
                    ledBuffer.setRGB(centerIndexOne - i, r, g, b);
                    ledBuffer.setRGB(centerIndexTwo + i, r, g, b);
                }
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }
        }
    }

    private void runningLightsFromEdges(final Color color) {
        if (ledBuffer.getLength() % 2 == 1) {
            int position = 0;
            for (int j = 0; j < ledBuffer.getLength(); j++) {
                if (patternChanged) {
                    return;
                }
                position++; // = 0; //Position + Rate;
                for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
                    // sine wave, 3 offset waves make a rainbow!
                    // float level = sin(i+Position) * 127 + 128;
                    // setPixel(i,level,0,0);
                    // float level = sin(i+Position) * 127 + 128;
                    int r = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.red * 255));
                    int g = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.green * 255));
                    int b = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.blue * 255));
                    ledBuffer.setRGB(i, r, g, b);
                    ledBuffer.setRGB((ledBuffer.getLength() - 1) - i, r, g, b);
                }
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }

            final int centerIndex = (ledBuffer.getLength() / 2);
            int r = (int) (((Math.sin(centerIndex + position) * 127 + 128) / 255) * (color.red * 255));
            int g = (int) (((Math.sin(centerIndex + position) * 127 + 128) / 255) * (color.green * 255));
            int b = (int) (((Math.sin(centerIndex + position) * 127 + 128) / 255) * (color.blue * 255));
            ledBuffer.setRGB(centerIndex, r, g, b);
            ledController.setData(ledBuffer);
            Timer.delay(delay);

        } else {
            int position = 0;
            for (int j = 0; j < ledBuffer.getLength(); j++) {
                if (patternChanged) {
                    return;
                }
                position++; // = 0; //Position + Rate;
                for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
                    // sine wave, 3 offset waves make a rainbow!
                    // float level = sin(i+Position) * 127 + 128;
                    // setPixel(i,level,0,0);
                    // float level = sin(i+Position) * 127 + 128;
                    int r = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.red * 255));
                    int g = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.green * 255));
                    int b = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.blue * 255));
                    ledBuffer.setRGB(i, r, g, b);
                    ledBuffer.setRGB((ledBuffer.getLength() - 1) - i, r, g, b);
                }
                ledController.setData(ledBuffer);
                Timer.delay(delay);
            }
        }
    }

    private void runningLightsFromStart(final Color color) {
        int position = 0;
        for (int j = 0; j < ledBuffer.getLength() * 2; j++) {
            if (patternChanged) {
                return;
            }
            position++; // = 0; //Position + Rate;
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                // sine wave, 3 offset waves make a rainbow!
                // float level = sin(i+Position) * 127 + 128;
                // setPixel(i,level,0,0);
                // float level = sin(i+Position) * 127 + 128;
                int r = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.red * 255));
                int g = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.green * 255));
                int b = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.blue * 255));
                ledBuffer.setRGB(i, r, g, b);
            }
            ledController.setData(ledBuffer);
            Timer.delay(delay);
        }
    }

    private void runningLightsFromEnd(final Color color) {
        int position = 0;
        for (int j = 0; j < ledBuffer.getLength() * 2; j++) {
            if (patternChanged) {
                return;
            }
            position++; // = 0; //Position + Rate;
            for (int i = ledBuffer.getLength() - 1; i >= 0; i--) {
                // sine wave, 3 offset waves make a rainbow!
                // float level = sin(i+Position) * 127 + 128;
                // setPixel(i,level,0,0);
                // float level = sin(i+Position) * 127 + 128;
                int r = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.red * 255));
                int g = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.green * 255));
                int b = (int) (((Math.sin(i + position) * 127 + 128) / 255) * (color.blue * 255));
                ledBuffer.setRGB(i, r, g, b);
            }
            ledController.setData(ledBuffer);
            Timer.delay(delay);
        }
    }

    private void staticColor(final Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        ledController.setData(ledBuffer);
    }

    private int findClosestMultiple(final int number, final float percent) {
        final int startNumber = Math.round(number * percent);
        int index = 0;
        while (true) {
            if (index >= startNumber) {
                break;
            }
            if ((number % (startNumber - index)) == 0) {
                return (startNumber - index);
            } else if ((number % (startNumber + index)) == 0) {
                return (startNumber + index);
            }
            index++;
        }
        return 1;
    }

    public void reset() {
        if (DriverStation.getInstance().isFMSAttached()) {
            switch (DriverStation.getInstance().getAlliance()) {
            case Red:
                setPattern(Pattern.Color_From_Center, Color.kGreen, Color.kRed);
                break;
            case Blue:
                setPattern(Pattern.Color_From_Center, Color.kGreen, Color.kBlue);
                break;
            default:
                setPattern(Pattern.Static_Color, Color.kGreen, Color.kGreen);
                break;
            }
        } else {
            setPattern(Pattern.Static_Color, Color.kGreen, Color.kGreen);
        }
        setDelay(.05);
    }

}