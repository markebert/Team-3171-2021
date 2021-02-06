package frc.team3171.auton;

/**
 * @author Mark Ebert
 * @author Charles Fee
 * @author Elijah Hoda
 */
public class AutonRecorderData {

    // Variables to store
    private double fpgaTimestamp, leftX, leftY, rightX, rightY;
    private boolean shooter, pickup, targetLock, quickTurn;

    /**
     * Constructor
     */
    public AutonRecorderData() {
        // Initializes all values to 0 and false
        fpgaTimestamp = 0;
        leftX = 0;
        leftY = 0;
        rightX = 0;
        rightY = 0;
        shooter = false;
        pickup = false;
        targetLock = false;
        quickTurn = false;
    }

    /**
     * Updates the fpgaTimestamp variable in this class to the value of the given
     * parameter.
     * 
     * @param fpgaTimestamp The new fpgaTimestamp variable to update the value from.
     */
    public void setFPGATimestamp(final double fpgaTimestamp) {
        this.fpgaTimestamp = fpgaTimestamp;
    }

    /**
     * Updates the leftX variable in this class to the value of the given parameter.
     * 
     * @param leftX The new leftX variable to update the value from.
     */
    public void setLeftX(final double leftX) {
        this.leftX = leftX;
    }

    /**
     * Updates the leftY variable in this class to the value of the given parameter.
     * 
     * @param leftY The new leftY variable to update the value from.
     */
    public void setLeftY(final double leftY) {
        this.leftY = leftY;
    }

    /**
     * Updates the rightX variable in this class to the value of the given
     * parameter.
     * 
     * @param rightX The new rightX variable to update the value from.
     */
    public void setRightX(final double rightX) {
        this.rightX = rightX;
    }

    /**
     * Updates the rightY variable in this class to the value of the given
     * parameter.
     * 
     * @param rightY The new rightY variable to update the value from.
     */
    public void setRightY(final double rightY) {
        this.rightY = rightY;
    }

    /**
     * Updates the shooter variable in this class to the value of the given
     * parameter.
     * 
     * @param shooter The new shooter variable to update the value from.
     */
    public void setShooter(final boolean shooter) {
        this.shooter = shooter;
    }

    /**
     * Updates the pickup variable in this class to the value of the given
     * parameter.
     * 
     * @param pickup The new pickup variable to update the value from.
     */
    public void setPickup(final boolean pickup) {
        this.pickup = pickup;
    }

    /**
     * Updates the targetLock variable in this class to the value of the given
     * parameter.
     * 
     * @param targetLock The new targetLock variable to update the value from.
     */
    public void setTargetLock(final boolean targetLock) {
        this.targetLock = targetLock;
    }

    /**
     * Updates the quickTurn variable in this class to the value of the given
     * parameter.
     * 
     * @param quickTurn The new quickTurn variable to update the value from.
     */
    public void setQuickTurn(final boolean quickTurn) {
        this.quickTurn = quickTurn;
    }

    /**
     * Returns the fpgaTimestamp variable stored in {@code AutonRecorderData}.
     * 
     * @return The current fpgaTimestamp variable value.
     */
    public double getFPGATimestamp() {
        return fpgaTimestamp;
    }

    /**
     * Returns the leftX variable stored in {@code AutonRecorderData}.
     * 
     * @return The current leftX variable value.
     */
    public double getLeftX() {
        return leftX;
    }

    /**
     * Returns the leftY variable stored in {@code AutonRecorderData}.
     * 
     * @return The current leftY variable value.
     */
    public double getLeftY() {
        return leftY;
    }

    /**
     * Returns the rightX variable stored in {@code AutonRecorderData}.
     * 
     * @return The current rightX variable value.
     */
    public double getRightX() {
        return rightX;
    }

    /**
     * Returns the rightY variable stored in {@code AutonRecorderData}.
     * 
     * @return The current rightY variable value.
     */
    public double getRightY() {
        return rightY;
    }

    /**
     * Returns the shooter variable stored in {@code AutonRecorderData}.
     * 
     * @return The current shooter variable value.
     */
    public boolean getShooter() {
        return shooter;
    }

    /**
     * Returns the pickup variable stored in {@code AutonRecorderData}.
     * 
     * @return The current pickup variable value.
     */
    public boolean getPickup() {
        return pickup;
    }

    /**
     * Returns the targetLock variable stored in {@code AutonRecorderData}.
     * 
     * @return The current targetLock variable value.
     */
    public boolean getTargetLock() {
        return targetLock;
    }

    /**
     * Returns the quickTurn variable stored in {@code AutonRecorderData}.
     * 
     * @return The current quickTurn variable value.
     */
    public boolean getQuickTurn() {
        return quickTurn;
    }

    /**
     * Checks if the provided new data is different from the current data, as long
     * as the new data's timestamp is later then the current timestamp, if so
     * returns true, otherwise false.
     * 
     * @param newData The new set of data containing the new values to compare to
     *                the current values.
     * @return Returns true if the new data is different, otherwise false.
     */
    public boolean isChanged(final AutonRecorderData newData) {
        if (newData.getFPGATimestamp() >= fpgaTimestamp) {
            if (newData.getLeftX() != leftX) {
                return true;
            } else if (newData.getLeftY() != leftY) {
                return true;
            } else if (newData.getRightX() != rightX) {
                return true;
            } else if (newData.getRightY() != rightY) {
                return true;
            } else if (newData.getQuickTurn() != quickTurn) {
                return true;
            } else if (newData.getShooter() != shooter) {
                return true;
            } else if (newData.getPickup() != pickup) {
                return true;
            } else if (newData.getTargetLock() != targetLock) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns the string representation of the data. Formatted as
     * fpgaTimestamp,leftX,leftY,rightX,rightY;\n
     */
    @Override
    public String toString() {
        return String.format("%.3f,%.2f,%.2f,%.2f,%.2f,%b,%b,%b,%b;\n", fpgaTimestamp, leftX, leftY, rightX, rightY,
                shooter, pickup, targetLock, quickTurn);
    }

    // takes the data from a string and seperates it into the seperate data values
    public static AutonRecorderData fromString(String dataString) {
        dataString = dataString.trim();
        if (dataString.endsWith(";")) {
            dataString = dataString.substring(0, dataString.length() - 1);
            final String[] data = dataString.split(",");
            if (data.length == 9) {
                final AutonRecorderData autonData = new AutonRecorderData();
                autonData.setFPGATimestamp(Double.parseDouble(data[0]));
                autonData.setLeftX(Double.parseDouble(data[1]));
                autonData.setLeftY(Double.parseDouble(data[2]));
                autonData.setRightX(Double.parseDouble(data[3]));
                autonData.setRightY(Double.parseDouble(data[4]));
                autonData.setShooter(Boolean.parseBoolean(data[5]));
                autonData.setPickup(Boolean.parseBoolean(data[6]));
                autonData.setTargetLock(Boolean.parseBoolean(data[7]));
                autonData.setQuickTurn(Boolean.parseBoolean(data[8]));
                return autonData;
            }
        }
        return null;
    }

}