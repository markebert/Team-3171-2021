package frc.team3171.drive;

// Java Imports
import java.util.ArrayList;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

/**
 * @author Mark Ebert
 */
public class TalonFXMotorGroup implements MotorGroup {

    // Motor Controllers
    private final TalonFX masterTalonFX;
    private final ArrayList<TalonFX> slaveMotorGroup;

    /**
     * Constructor
     * 
     * @param inverted   Whether or not the direction of the {@link TalonFX} motors
     *                   need to be inverted.
     * @param canIDArray An int array containing the CAN IDs of the {@link TalonFX}
     *                   motors.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors in the canIDArray.
     */
    public TalonFXMotorGroup(final boolean inverted, final int... canIDArray) throws Exception {
        if (canIDArray.length == 0) {
            throw new Exception("Invalid amount of motors provided to the motor group!");
        }

        // Creates an ArrayList to hold all of the slave TalonFX's
        slaveMotorGroup = new ArrayList<>(canIDArray.length - 1);

        // Init the master TalonFX
        masterTalonFX = new TalonFX(canIDArray[0]);
        masterTalonFX.configFactoryDefault();
        masterTalonFX.setInverted(inverted);
        masterTalonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);

        // Assigns a new TalonFX to the ArrayList using the canID from the canIDArray
        for (int i = 1; i < canIDArray.length; i++) {
            final TalonFX talonFX = new TalonFX(canIDArray[i]);
            talonFX.configFactoryDefault();
            talonFX.follow(masterTalonFX);
            talonFX.setInverted(InvertType.FollowMaster);
            slaveMotorGroup.add(talonFX);
        }
    }

    /**
     * Constructor
     * 
     * @param canIDArray An int array containing the CAN IDs of the {@link TalonFX}
     *                   motors.
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors in the canIDArray.
     */
    public TalonFXMotorGroup(final int[] canIDArray) throws Exception {
        this(false, canIDArray);
    }

    /**
     * Sets all of the {@link TalonFX} motors in the {@linkplain TalonFXMotorGroup}
     * to the desired speed.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the {@linkplain TalonFX}
     *              motors to.
     */
    @Override
    public void set(final double speed) {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * given value
         */
        masterTalonFX.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets whether or not the direction of the {@link TalonFX} motors in the
     * {@linkplain TalonFXMotorGroup} need to be inverted.
     * 
     * @param inverted Whether or not the direction of the {@link TalonFX} motors
     *                 need to be inverted.
     */
    @Override
    public void setInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonFX, and therefore it's
         * followers, need to be inverted
         */
        masterTalonFX.setInverted(inverted);
    }

    /**
     * Gets whether or not the direction of the {@link TalonFX} motors in the
     * {@linkplain TalonFXMotorGroup} are inverted.
     * 
     * @return True, if the motors are inverted, false otherwise.
     */
    @Override
    public boolean getInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's
         * followers, are inverted
         */
        return masterTalonFX.getInverted();
    }

    /**
     * Returns the raw value of the {@link TalonFX} integrated encoder. The encoder
     * has 2048 ticks per revolution.
     * 
     * @return The raw value of the {@link TalonFX} integrated encoder.
     */
    public int getEncoderValue() {
        return (int) masterTalonFX.getSelectedSensorPosition();
    }

    /**
     * Returns the velocity of the {@link TalonFX} integrated encoder. The encoder
     * has 2048 ticks per revolution and the return units of the velocity is in
     * ticks per 100ms.
     * 
     * @return The velocity, in ticks per 100ms, of the {@link TalonFX} integrated
     *         encoder.
     */
    public int getEncoderVelocity() {
        return (int) masterTalonFX.getSelectedSensorVelocity();
    }

    /**
     * Disables all of the {@link TalonFX} motors in the
     * {@linkplain TalonFXMotorGroup}.
     */
    @Override
    public void disable() {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * value of 0 and disables it
         */
        masterTalonFX.set(ControlMode.Disabled, 0);
    }

}