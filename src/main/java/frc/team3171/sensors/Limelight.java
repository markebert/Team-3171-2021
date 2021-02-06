package frc.team3171.sensors;

// FRC Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * @author Mark Ebert
 */
public class Limelight {

    // Network Table representing the Limelight
    private final NetworkTable limelightNetworkTable;

    /**
     * Constructor
     */
    public Limelight() {
        limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
        limelightNetworkTable.getEntry("ledMode").setNumber(3);
        limelightNetworkTable.getEntry("camMode").setNumber(0);
        limelightNetworkTable.getEntry("stream").setNumber(0);
    }

    /**
     * Turns off the build in LEDs on the Liemlight.
     */
    public void turnLightOff() {
        limelightNetworkTable.getEntry("ledMode").setNumber(1);
    }

    /**
     * Turns on the build in LEDs on the Liemlight.
     */
    public void turnLightOn() {
        limelightNetworkTable.getEntry("ledMode").setNumber(3);
    }

    /**
     * 
     * @return true for if the Limelight has valid targets, false otherwise.
     */
    public boolean hasTarget() {
        return limelightNetworkTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
     *         degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getTargetHorizontalOffset() {
        return limelightNetworkTable.getEntry("tx").getDouble(0);
    }

    /**
     * @return Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
     *         degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getTargetVerticalOffset() {
        return limelightNetworkTable.getEntry("ty").getDouble(0);
    }

}