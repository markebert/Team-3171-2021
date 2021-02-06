package frc.team3171;

/**
 * @author Mark Ebert
 */
public class HelperFunctions {

	/**
	 * This function returns a value adjusted for the inputed deadzone.
	 * 
	 * @param deadzoneRange The range that will be used when adjusting the inputed
	 *                      value.
	 * @param currentValue  The original unaltered value that should range from -1.0
	 *                      to 1.0.
	 * @return The value adjusted for the given deadzone range.
	 */
	public static double Deadzone(double deadzoneRange, final double currentValue) {
		deadzoneRange = Math.abs(deadzoneRange);
		if (currentValue <= -deadzoneRange || currentValue >= deadzoneRange) {
			return currentValue;
		} else {
			return 0;
		}
	}

	/**
	 * This function returns a value adjusted for the inputed deadzone and maps it
	 * to the full range.
	 * 
	 * @param deadzoneRange The range that will be used when adjusting the inputed
	 *                      value.
	 * @param currentValue  The original unaltered value that should range from -1.0
	 *                      to 1.0.
	 * @return The value adjusted for the given deadzone range.
	 */
	public static double Deadzone_With_Map(double deadzoneRange, final double currentValue) {
		deadzoneRange = Math.abs(deadzoneRange);
		if (currentValue <= -deadzoneRange || currentValue >= deadzoneRange) {
			if (currentValue > 0) {
				return Map(currentValue, deadzoneRange, 1.0, 0.0, 1.0);
			}
			return Map(currentValue, -1.0, -deadzoneRange, -1.0, 0.0);
		}
		return 0;
	}

	/**
	 * Maps the value from the first range to the second range.
	 * 
	 * @param x:       the number to map.
	 * @param in_min:  the lower bound of the value's current range.
	 * @param in_max:  the upper bound of the value's current range.
	 * @param out_min: the lower bound of the value's target range.
	 * @param out_max: the upper bound of the value's target range.
	 */
	public static int Map(int x, int in_min, int in_max, int out_min, int out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	/**
	 * Maps the value from the first range to the second range.
	 * 
	 * @param x:       the number to map.
	 * @param in_min:  the lower bound of the value's current range.
	 * @param in_max:  the upper bound of the value's current range.
	 * @param out_min: the lower bound of the value's target range.
	 * @param out_max: the upper bound of the value's target range.
	 */
	public static double Map(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	/**
	 * Determines if the currentValue is within range of the desiredValue by the
	 * given percentage.
	 * 
	 * @param currentValue    The current value to compare.
	 * @param desiredValue    The desired value to compare the current value to.
	 * @param errorPercentage The desired percent error margin allowed for the
	 *                        calculation.
	 * @return true if the currentValue is within the desiredValue by the given
	 *         percent margin, false otherwise.
	 */
	public static boolean Within_Percent_Error(final double currentValue, final double desiredValue,
			final double errorPercentage) {
		final double acceptableError = Math.abs(desiredValue) * Math.abs(errorPercentage);
		if ((currentValue >= (desiredValue - acceptableError)) && (currentValue <= (desiredValue + acceptableError))) {
			return true;
		}
		return false;
	}

	/**
	 * Turns a gyros raw values into angles that are more useful (-180.0 degrees to
	 * 180.0 degrees), where 0 is the front and -180.0 is 180 degrees
	 * counterclockwise from 0 and 180.0 is 180 degrees clockwise from 0.
	 * 
	 * @param gyroValue The gyro value to normalize.
	 * 
	 * @return Returns the Gyro's angle in a normalized format, from -180 degrees to
	 *         180 degrees.
	 */
	public static double Normalize_Gryo_Value(final double gyroValue) {
		final double gyroAngle = gyroValue % 360;
		if (gyroAngle > 180) {
			return gyroAngle - 360;
		} else if (gyroAngle < -180) {
			return gyroAngle + 360;
		}
		return gyroAngle;
	}

	/**
	 * Returns the shortest gyro displacement in the best direction from the
	 * currentValue to the desiredValue. This function will normalize, scale the
	 * values from -180.0 degrees to 180.0 degrees, both the currentValue and the
	 * desiredValue and will return the displacement as a normalized value as well.
	 * 
	 * @param currentValue The current gyro value in degrees.
	 * @param desiredValue The desired gyro value in dgerees.
	 * @return The shortest gyro displacement representing the best direction to
	 *         reach the desired value with the least amount of turning.
	 */
	public static double Get_Gyro_Displacement(final double currentValue, final double desiredValue) {
		final double displacement = Normalize_Gryo_Value(desiredValue) - Normalize_Gryo_Value(currentValue);
		if (displacement < -180) {
			return displacement + 360;
		} else if (displacement > 180) {
			return displacement - 360;
		}
		return displacement;
	}

}