package frc.team3171.controllers;

// Rev Robotics Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.sensors.WheelColorSensor;
import frc.team3171.sensors.WheelColorSensor.WheelColor;

/**
 * @author Mark Ebert
 */
public class Wheel implements RobotProperties {

    // Wheel Motor
    private final CANSparkMax wheelMotor;

    private WheelColor phaseOneStartingColor = null, phaseTwoStartingColor = null;
    private volatile int phaseOneCount = 0;
    private volatile boolean phaseOneEdgeTrigger = false;

    /**
     * Constructor
     * 
     * @param colorSensor The {@linkplain WheelColorSensor} that will control how
     *                    far the wheel spins.
     */
    public Wheel(final WheelColorSensor colorSensor) {
        // Init the motors
        wheelMotor = new CANSparkMax(wheelCANID, MotorType.kBrushless);
        wheelMotor.restoreFactoryDefaults();
        wheelMotor.setIdleMode(IdleMode.kBrake);
        wheelMotor.setInverted(wheelInverted);
    }

    /**
     * Sets the speed of the wheel motor to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the wheel motor to.
     */
    public void setWheelSpeed(final double speed) {
        wheelMotor.set(speed);
    }

    /**
     * Stops the wheel from spinning.
     */
    public void stopWheel() {
        wheelMotor.set(0);
        phaseOneStartingColor = null;
        phaseTwoStartingColor = null;
    }

    /**
     * Spins the wheel the required amount of revolutions for phase one of the
     * wheel.
     * 
     * @param currentColor The current {@linkplain WheelColor} detected by the
     *                     {@linkplain WheelColorSensor}.
     */
    public boolean wheelPhaseOne(WheelColor currentColor) {
        if (phaseOneStartingColor == null) {
            phaseOneStartingColor = currentColor;
            phaseOneEdgeTrigger = true;
            phaseOneCount = 1;
            setWheelSpeed(.2);
        } else {
            if ((phaseOneStartingColor == currentColor) && !phaseOneEdgeTrigger) {
                phaseOneCount++;
                if (phaseOneCount >= 8) {
                    stopWheel();
                    return true;
                }
            }
            phaseOneEdgeTrigger = phaseOneStartingColor.equals(currentColor);
        }
        return false;
    }

    /**
     * Spins the wheel to the designated color for phase two.
     * 
     * @param currentColor The current {@linkplain WheelColor} detected by the
     *                     {@linkplain WheelColorSensor}.
     * @param desiredColor The desired {@linkplain WheelColor} to spin the wheel to.
     */
    public boolean wheelPhaseTwo(WheelColor currentColor, WheelColor desiredColor) {
        if (phaseTwoStartingColor == null) {
            phaseTwoStartingColor = currentColor;
            boolean alternateDirection = false;
            if (phaseTwoStartingColor == WheelColor.kBlueTarget && desiredColor == WheelColor.kGreenTarget) {
                alternateDirection = true;
            } else if (phaseTwoStartingColor == WheelColor.kGreenTarget && desiredColor == WheelColor.kRedTarget) {
                alternateDirection = true;
            } else if (phaseTwoStartingColor == WheelColor.kRedTarget && desiredColor == WheelColor.kYellowTarget) {
                alternateDirection = true;
            } else if (phaseTwoStartingColor == WheelColor.kYellowTarget && desiredColor == WheelColor.kBlueTarget) {
                alternateDirection = true;
            }
            if (alternateDirection) {
                setWheelSpeed(-.075);
            } else {
                setWheelSpeed(.075);
            }
        } else if (currentColor == desiredColor) {
            stopWheel();
            return true;
        }
        return false;
    }

    /**
     * Disables all motors in the {@linkplain Wheel} class.
     */
    public void disable() {
        wheelMotor.disable();
    }

}