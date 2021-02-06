package frc.team3171.drive;

/**
 * @author Mark Ebert
 */
public class TractionDrive {

	// Motor Groups
	private final MotorGroup leftMotorGroup, rightMotorGroup;

	// Drive Direction
	private volatile boolean driveDirectionFlipped;

	/**
	 * Constructor
	 * 
	 * @param leftMotorGroup  A {@link MotorGroup} containing all of the motors used
	 *                        for the left side of the drive train.
	 * @param rightMotorGroup A {@link MotorGroup} containing all of the motors used
	 *                        for the right side of the drive train.
	 * @param inverted        Whether or not the direction of the drive train needs
	 *                        to be inverted.
	 */
	public TractionDrive(final MotorGroup leftMotorGroup, final MotorGroup rightMotorGroup, final boolean inverted) {
		leftMotorGroup.setInverted(inverted);
		rightMotorGroup.setInverted(!inverted);
		this.leftMotorGroup = leftMotorGroup;
		this.rightMotorGroup = rightMotorGroup;
		driveDirectionFlipped = false;
	}

	/**
	 * Constructor
	 * 
	 * @param leftMotorGroup  A {@link MotorGroup} containing all of the motors used
	 *                        for the left side of the drive train.
	 * @param rightMotorGroup A {@link MotorGroup} containing all of the motors used
	 *                        for the right side of the drive train.
	 */
	public TractionDrive(final MotorGroup leftMotorGroup, final MotorGroup rightMotorGroup) {
		this(leftMotorGroup, rightMotorGroup, false);
	}

	/**
	 * Standard Tank Traction Drive.
	 * 
	 * @param leftSickY   The speed value of the left motors based the left
	 *                    {@code Joystick} y-axis value.
	 * @param rightStickY The speed value of the left motors based the right
	 *                    {@code Joystick} y-axis value.
	 */
	public void tankTraction(final double leftSickY, final double rightStickY) {
		if (driveDirectionFlipped) {
			leftMotorGroup.set(-leftSickY);
			rightMotorGroup.set(rightStickY);
		} else {
			leftMotorGroup.set(leftSickY);
			rightMotorGroup.set(-rightStickY);
		}
	}

	/**
	 * Standard Traction Drive using a Mecanum control scheme.
	 * 
	 * @param leftStickY  The speed value of the motors based the left
	 *                    {@code Joystick} y-axis value.
	 * @param rightStickX The speed value of the motors based the right
	 *                    {@code Joystick} x-axis value.
	 */
	public void mecanumTraction(final double leftStickY, final double rightStickX) {
		if (driveDirectionFlipped) {
			leftMotorGroup.set(-leftStickY + rightStickX);
			rightMotorGroup.set(-leftStickY - rightStickX);
		} else {
			leftMotorGroup.set(leftStickY + rightStickX);
			rightMotorGroup.set(leftStickY - rightStickX);
		}
	}

	/**
	 * Sets whether or not the drive direction of the robot needs to be flipped.
	 * 
	 * @param flipped Whether ot not to flip the drive direction of the robot.
	 */
	public void setDriveDirectionFlipped(final boolean flipped) {
		driveDirectionFlipped = flipped;
	}

	/**
	 * Toggles whether or not the drive direction of the robot is flipped.
	 */
	public void toggleDriveDirectionFlipped() {
		driveDirectionFlipped = !driveDirectionFlipped;
	}

	/**
	 * Returns if the drive direction of the robot is currently flipped.
	 * 
	 * @return True if the drive direction if flipped, false otherwise.
	 */
	public boolean getDriveDirectionFlipped() {
		return driveDirectionFlipped;
	}

	/**
	 * Disables all of the motors used in the drive train.
	 */
	public void disable() {
		leftMotorGroup.disable();
		rightMotorGroup.disable();
	}

}
