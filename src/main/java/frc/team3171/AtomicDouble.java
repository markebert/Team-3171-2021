package frc.team3171;

// Java Imports
import java.util.concurrent.atomic.AtomicLong;
import static java.lang.Double.doubleToLongBits;
import static java.lang.Double.longBitsToDouble;

public class AtomicDouble extends Number {

    /**
     * Generated Serial Version ID
     */
    private static final long serialVersionUID = -7705013431992390822L;
    private final AtomicLong bits;

    /**
     * Constructor, initializes the {@code AtomicDouble} to 0.
     */
    public AtomicDouble() {
        this(0f);
    }

    /**
     * Constructor, initializes the {@code AtomicDouble} to the given value.
     * 
     * @param initialValue The initial value to initialize the {@code AtomicDouble}
     *                     as.
     */
    public AtomicDouble(final double initialValue) {
        bits = new AtomicLong(doubleToLongBits(initialValue));
    }

    public final boolean compareAndSet(final double expect, final double update) {
        return bits.compareAndSet(doubleToLongBits(expect), doubleToLongBits(update));
    }

    /**
     * Sets the current value of the {@code AtomicDouble}.
     * 
     * @param newValue The new value to set.
     */
    public final void set(final double newValue) {
        bits.set(doubleToLongBits(newValue));
    }

    /**
     * Returns the double value stored in this {@code AtomicDouble}.
     * 
     * @return The double value.
     */
    public final double get() {
        return longBitsToDouble(bits.get());
    }

    /**
     * Returns the value of the {@code AtomicDouble} as a float. Note, there may be
     * rounding errors or data may be lost.
     */
    public float floatValue() {
        return (float) get();
    }

    /**
     * Returns the double value stored in this {@code AtomicDouble} and then updates
     * the stored value to the given value.
     * 
     * @param newValue The new double value to set.
     * @return The original double value.
     */
    public final double getAndSet(final float newValue) {
        return longBitsToDouble(bits.getAndSet(doubleToLongBits(newValue)));
    }

    /**
     * Returns the double value stored in the {@code AtomicDouble}. Same as get().
     */
    public double doubleValue() {
        return get();
    }

    /**
     * Returns the value of the {@code AtomicDouble} as a int. Note, there may be
     * rounding errors or data may be lost.
     */
    public int intValue() {
        return (int) get();
    }

    /**
     * Returns the value of the {@code AtomicDouble} as a long. Note, there may be
     * rounding errors or data may be lost.
     */
    public long longValue() {
        return (long) get();
    }

}