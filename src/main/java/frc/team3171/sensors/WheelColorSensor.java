package frc.team3171.sensors;

// Java Imports
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

// FRC Imports
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

// Rev Imports
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

/**
 * @author Mark Ebert
 */
public class WheelColorSensor {

    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
     * The device will be automatically initialized with default parameters.
     */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Match object is used to register and detect known colors. This
     * can be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private final ColorMatch m_colorMatcher = new ColorMatch();

    // Most recent color sensor data
    private volatile double redValue, greenValue, blueValue, confidenceValue;
    private volatile WheelColor color;

    // Executor Service
    private final ScheduledExecutorService es;

    /**
     * Enums to represent the colors on the color wheel.
     */
    public enum WheelColor {

        kBlueTarget(0.245, 0.453, 0.3015), kGreenTarget(0.277, 0.507, 0.2148), kRedTarget(0.390, 0.433, 0.176),
        kYellowTarget(0.326, 0.518, 0.154);

        private Color color;

        private WheelColor(double red, double green, double blue) {
            this.color = ColorMatch.makeColor(red, green, blue);
        }

        public Color color() {
            return color;
        }

    }

    /**
     * Constructor
     */
    public WheelColorSensor() {
        m_colorMatcher.addColorMatch(WheelColor.kRedTarget.color());
        m_colorMatcher.addColorMatch(WheelColor.kGreenTarget.color());
        m_colorMatcher.addColorMatch(WheelColor.kBlueTarget.color());
        m_colorMatcher.addColorMatch(WheelColor.kYellowTarget.color());

        es = Executors.newSingleThreadScheduledExecutor();
        es.scheduleAtFixedRate(() -> {
            final Color currentColor = m_colorSensor.getColor();
            redValue = currentColor.red;
            greenValue = currentColor.green;
            blueValue = currentColor.blue;

            final ColorMatchResult matchResult = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
            confidenceValue = matchResult.confidence;
            if (matchResult.color.equals(WheelColor.kBlueTarget.color())) {
                color = WheelColor.kBlueTarget;
            } else if (matchResult.color.equals(WheelColor.kRedTarget.color())) {
                color = WheelColor.kRedTarget;
            } else if (matchResult.color.equals(WheelColor.kGreenTarget.color())) {
                color = WheelColor.kGreenTarget;
            } else if (matchResult.color.equals(WheelColor.kYellowTarget.color())) {
                color = WheelColor.kYellowTarget;
            }
        }, 0, 100, TimeUnit.MILLISECONDS);
    }

    /**
     * 
     * @return
     */
    public double redValue() {
        return redValue;
    }

    /**
     * 
     * @return
     */
    public double greenValue() {
        return greenValue;
    }

    /**
     * 
     * @return
     */
    public double blueValue() {
        return blueValue;
    }

    /**
     * @return
     */
    public double getConfidenceValue() {
        return confidenceValue;
    }

    /**
     * 
     * @return
     */
    public WheelColor getCurrentColor() {
        return color;
    }

}
