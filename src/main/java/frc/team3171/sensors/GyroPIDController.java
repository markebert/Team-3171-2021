package frc.team3171.sensors;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

// Team 3171 Imports
import static frc.team3171.HelperFunctions.Get_Gyro_Displacement;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class GyroPIDController {

    /**
     * Objects
     */
    private final DoubleSupplier sensor;
    private final ScheduledExecutorService es;
    private final ReentrantLock START_LOCK, PID_LOCK;
    private final AtomicBoolean started;

    /**
     * Variables
     */
    private volatile double pidValue, sensorValue, sensorLockValue;
    private double sum = 0, rate = 0;
    private double proportionalTemp, sumTemp, rateTemp;
    private long currentTime = 0, lastTime = 0;
    private volatile boolean disablePID = true;

    /**
     * Constants
     */
    private volatile double kP, kI, kD;
    private final double PID_MIN, PID_MAX;

    /**
     * Constructor
     * 
     * @param sensor  The sensor that the {@code PIDController} will get its sensor
     *                values from. This object must implement the {@code Sensor}
     *                Interface. Once set, this {@code DoubleSupplier} cannot be
     *                changed during this instance.
     * @param kP      The proportional value constant for the {@code PIDController}.
     *                This value is arbitrary and based upon testing of the
     *                {@code PIDController} to find the values that works best for
     *                your system. Once set, this values cannot be changed during
     *                this instance.
     * @param kI      The Integral value constant for the {@code PIDController}.
     *                This value is arbitrary and based upon testing of the
     *                {@code PIDController} to find the values that works best for
     *                your system. Once set, this value cannot be changed during
     *                this instance.
     * @param kD      The Derivative value constant for the {@code PIDController}.
     *                This value is arbitrary and based upon testing of the
     *                {@code PIDController} to find the values that works best for
     *                your system. Once set, this value cannot be changed during
     *                this instance.
     * @param PID_MIN The minimum value that the {@code PIDController} is allowed
     *                obtain. If the value becomes smaller then the given constant,
     *                then the {@code PIDController} will also prevent accumulation
     *                of the sum and will set the PID value to the minimum value.
     *                Once set, this value cannot be changed during this instance.
     * @param PID_MAX The maximum value that the {@code PIDController} is allowed
     *                obtain. If the value becomes larger then the given constant,
     *                then the {@code PIDController} will also prevent accumulation
     *                of the sum and will set the PID value to the maximum value.
     *                Once set, this value cannot be changed during this instance.
     */
    public GyroPIDController(DoubleSupplier sensor, double kP, double kI, double kD, double PID_MIN, double PID_MAX) {
        this.sensor = sensor;
        this.es = Executors.newSingleThreadScheduledExecutor();
        this.START_LOCK = new ReentrantLock(true);
        this.PID_LOCK = new ReentrantLock(true);
        this.started = new AtomicBoolean();

        this.pidValue = 0;
        this.sensorValue = 0;
        this.sensorLockValue = 0;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.PID_MIN = PID_MIN;
        this.PID_MAX = PID_MAX;
    }

    /**
     * Starts the PID Controller. Can only be called once.
     * 
     * @param updateRate  The update rate, in milliseconds of the pid controller.
     * @param defaultZero Whether or not when the {@link GyroPIDController} is
     *                    disabled if the sensor lock value should be set to 0 or to
     *                    the current sensor value.
     * @param logData     A queue to store the generated values from the controller
     *                    to graph or record.
     */
    public void start(final int updateRate, final boolean defaultZero, final ConcurrentLinkedQueue<String> logData) {
        try {
            START_LOCK.lock();
            final long startTime = System.currentTimeMillis();
            if (started.compareAndSet(false, true)) {
                es.scheduleAtFixedRate(() -> {
                    sensorValue = sensor.getAsDouble();
                    if (disablePID) {
                        if (defaultZero) {
                            updateSensorLockValue(0);
                        } else {
                            updateSensorLockValue();
                        }
                        if (logData != null) {
                            logData.add(String.format("%d,%.2f,%.2f,0,0,0,0", (System.currentTimeMillis() - startTime),
                                    sensorValue, sensorLockValue));
                        }
                    } else {
                        try {
                            PID_LOCK.lock();
                            pidValue = calculatePID(Get_Gyro_Displacement(sensorValue, sensorLockValue));
                        } finally {
                            PID_LOCK.unlock();
                        }
                        if (logData != null) {
                            logData.add(String.format("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                                    (System.currentTimeMillis() - startTime), getSensorValue(), getSensorLockValue(),
                                    getPIDValue(), proportionalTemp, sumTemp, rateTemp));
                        }
                    }
                }, 0, updateRate, TimeUnit.MILLISECONDS);
            }
        } finally {
            START_LOCK.unlock();
        }
    }

    /**
     * Starts the PID Controller. Can only be called once.
     */
    public void start(final boolean defaultZero) {
        start(20, defaultZero, null);
    }

    /**
     * Starts the PID Controller. Can only be called once.
     */
    public void start(final ConcurrentLinkedQueue<String> logData) {
        start(20, false, logData);
    }

    /**
     * Starts the PID Controller. Can only be called once.
     */
    public void start(final int updateRate) {
        start(updateRate, false, null);
    }

    /**
     * Starts the PID Controller. Can only be called once.
     */
    public void start() {
        start(20, false, null);
    }

    /**
     * Calculates the PID Value
     * 
     * @return The PID Value
     */
    private final double calculatePID(final double displacement) {
        double pid = 0, sum = this.sum;
        currentTime = System.currentTimeMillis();
        rate = (displacement / (currentTime - lastTime));
        sum += rate;
        proportionalTemp = (kP * displacement);
        sumTemp = (kI * sum);
        rateTemp = (kD * rate);
        pid = proportionalTemp + sumTemp + rateTemp;
        lastTime = currentTime;
        // Don't let the PID value increase past PID_MAX or below PID_MIN and
        // prevent accumulation of the sum
        if (pid >= PID_MAX) {
            pid = PID_MAX;
        } else if (pid <= PID_MIN) {
            pid = PID_MIN;
        } else {
            this.sum = sum;
        }
        return pid;
    }

    /**
     * Forces the PID Controller to set its current desired lock value to its
     * current sensor value.
     */
    public void updateSensorLockValue() {
        updateSensorLockValue(sensorValue);
    }

    /**
     * Forces the PID Controller to set its current desired lock value to the given
     * value but doesn't reset any accumulated values.
     * 
     * @param sensorLockValue The desired sensor lock value.
     */
    public void updateSensorLockValueWithoutReset(final double sensorLockValue) {
        this.sensorLockValue = Normalize_Gryo_Value(sensorLockValue);
    }

    /**
     * Forces the PID Controller to set its current desired lock value to the given
     * value.
     * 
     * @param sensorLockValue The desired sensor lock value.
     */
    public void updateSensorLockValue(final double sensorLockValue) {
        try {
            PID_LOCK.lock();
            this.sensorLockValue = Normalize_Gryo_Value(sensorLockValue);
            this.pidValue = 0;
            this.sum = 0;
            this.rate = 0;
            this.lastTime = System.currentTimeMillis();
        } finally {
            PID_LOCK.unlock();
        }
    }

    /**
     * Returns the PID Value
     * 
     * @return The current PID Value
     */
    public double getPIDValue() {
        return pidValue;
    }

    /**
     * Returns the current sensor value from the sensor associated with this
     * {@code PIDController}.
     * 
     * @return The current sensor reading from {@code Sensor}.
     */
    public double getSensorValue() {
        return sensorValue;
    }

    /**
     * Returns the current sensor lock value from this {@code PIDController}.
     * 
     * @return The current sensor lock value.
     */
    public double getSensorLockValue() {
        return sensorLockValue;
    }

    /**
     * Disables the {@code PIDController} such that the the PID value is zero and
     * constantly resets any accumulated values and sets the lock value to the
     * current sensor value.
     */
    public void disablePID() {
        this.disablePID = true;
    }

    /**
     * Re-enables the {@code PIDController} such that it starts calculating the PID
     * value again.
     */
    public void enablePID() {
        this.disablePID = false;
    }

    /**
     * Returns if the {@code PIDController} is disabled.
     */
    public boolean isDisabled() {
        return disablePID;
    }

    /**
     * Returns if the {@code PIDController} is enabled.
     */
    public boolean isEnabled() {
        return !disablePID;
    }

    /**
     * @return the kP
     */
    public double getkP() {
        return kP;
    }

    /**
     * @param kP the kP to set
     */
    public void setkP(double kP) {
        this.kP = kP;
    }

    /**
     * @return the kI
     */
    public double getkI() {
        return kI;
    }

    /**
     * @param kI the kI to set
     */
    public void setkI(double kI) {
        this.kI = kI;
    }

    /**
     * @return the kD
     */
    public double getkD() {
        return kD;
    }

    /**
     * @param kD the kD to set
     */
    public void setkD(double kD) {
        this.kD = kD;
    }

}