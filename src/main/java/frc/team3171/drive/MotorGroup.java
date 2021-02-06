package frc.team3171.drive;

/**
 * @author Mark Ebert
 */
public interface MotorGroup {

    /**
     * Sets all of the motors in the {@linkplain MotorGroup} to the desired speed.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the motors to.
     */
    public abstract void set(final double speed);

    /**
     * Sets whether or not the direction of the motors need to be inverted.
     * 
     * @param inverted Whether or not the direction of the motors need to be
     *                 inverted.
     */
    public abstract void setInverted(final boolean inverted);

    /**
     * Gets whether or not the direction of the motors are inverted.
     * 
     * @return True, if the motors are inverted, false otherwise.
     */
    public abstract boolean getInverted();

    /**
     * Disables all of the motors in the {@linkplain MotorGroup}.
     */
    public abstract void disable();

}