package frc.team3171.drive;

// Java Imports
import java.util.ArrayList;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;

/**
 * @author Mark Ebert
 */
public class TalonSRXMotorGroup implements MotorGroup {

    // Motor Controllers
    private final TalonSRX masterTalonSRX;
    private final ArrayList<TalonSRX> slaveMotorGroup;

    /**
     * Constructor
     * 
     * @param inverted   Whether or not the direction of the {@link TalonSRX} motors
     *                   need to be inverted.
     * @param canIDArray An int array containing the CAN IDs of the {@link TalonSRX}
     *                   motors.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors in the canIDArray.
     */
    public TalonSRXMotorGroup(final boolean inverted, final int... canIDArray) throws Exception {
        if (canIDArray.length == 0) {
            throw new Exception("Invalid amount of motors provided to the motor group!");
        }

        // Creates an ArrayList to hold all of the slave TalonSRX's
        slaveMotorGroup = new ArrayList<>(canIDArray.length - 1);

        // Init the master TalonSRX
        masterTalonSRX = new TalonSRX(canIDArray[0]);
        masterTalonSRX.configFactoryDefault();
        masterTalonSRX.setInverted(inverted);

        // Assigns a new TalonSRX to the ArrayList using the canID from the canIDArray
        for (int i = 1; i < canIDArray.length; i++) {
            final TalonSRX talonSRX = new TalonSRX(canIDArray[i]);
            talonSRX.configFactoryDefault();
            talonSRX.follow(masterTalonSRX);
            talonSRX.setInverted(InvertType.FollowMaster);
            slaveMotorGroup.add(talonSRX);
        }
    }

    /**
     * Constructor
     * 
     * @param canIDArray An int array containing the CAN IDs of the {@link TalonSRX}
     *                   motors.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors in the canIDArray.
     */
    public TalonSRXMotorGroup(final int... canIDArray) throws Exception {
        this(false, canIDArray);
    }

    /**
     * Sets all of the {@link TalonSRX} motors in the
     * {@linkplain TalonSRXMotorGroup} to the desired speed.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the {@linkplain TalonSRX}
     *              motors to.
     */
    @Override
    public void set(final double speed) {
        /*
         * Sets the speed of the master TalonSRX, and therefore it's followers, to the
         * given value
         */
        masterTalonSRX.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets whether or not the direction of the {@link TalonSRX} motors in the
     * {@linkplain TalonSRXMotorGroup} need to be inverted.
     * 
     * @param inverted Whether or not the direction of the {@link TalonSRX} motors
     *                 need to be inverted.
     */
    @Override
    public void setInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonSRX, and therefore it's
         * followers, need to be inverted
         */
        masterTalonSRX.setInverted(inverted);
    }

    /**
     * Gets whether or not the direction of the {@link TalonSRX} motors in the
     * {@linkplain TalonSRXMotorGroup} are inverted.
     * 
     * @return True, if the motors are inverted, false otherwise.
     */
    @Override
    public boolean getInverted() {
        /*
         * Gets whether or not the direction of the master TalonSRX, and therefore it's
         * followers, are inverted
         */
        return masterTalonSRX.getInverted();
    }

    /**
     * Disables all of the {@link TalonSRX} motors in the
     * {@linkplain TalonSRXMotorGroup}.
     */
    @Override
    public void disable() {
        /*
         * Sets the speed of the master TalonSRX, and therefore it's followers, to the
         * value of 0 and disables it
         */
        masterTalonSRX.set(ControlMode.Disabled, 0);
    }

}