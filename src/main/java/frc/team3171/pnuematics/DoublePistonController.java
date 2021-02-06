package frc.team3171.pnuematics;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

// FRC Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * @author Mark Ebert
 */
public class DoublePistonController {

    // Solenoids
    private final DoubleSolenoid doubleSolenoid;

    // Sets if the piston control is inverted or not
    private final boolean inverted;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean executorActive;

    /**
     * Constructor
     * 
     * @param doubleSolenoid The {@linkplain DoubleSolenoid} used for extending and
     *                       retracting the piston.
     * @param inverted       Whether or not the {@linkplain DoublePistonController}
     *                       needs to be inverted.
     */
    public DoublePistonController(final DoubleSolenoid doubleSolenoid, final boolean inverted) {
        this.doubleSolenoid = doubleSolenoid;
        this.inverted = inverted;
        this.executorService = Executors.newSingleThreadExecutor();
        this.executorLock = new ReentrantLock(true);
        this.executorActive = new AtomicBoolean();
    }

    /**
     * Constructor
     * 
     * @param pcmCANID       The CAN ID of the PCM.
     * @param forwardChannel The forward channel on the module to control (0-7).
     * @param reverseChannel The reverse channel on the module to control (0-7).
     * @param inverted       Whether or not the {@linkplain DoublePistonController}
     *                       needs to be inverted.
     */
    public DoublePistonController(final int pcmCANID, final int forwardChannel, final int reverseChannel,
            final boolean inverted) {
        this(new DoubleSolenoid(pcmCANID, forwardChannel, reverseChannel), inverted);
    }

    /**
     * Constructor
     * 
     * @param doubleSolenoid The {@linkplain DoubleSolenoid} used for extending and
     *                       retracting the piston.
     */
    public DoublePistonController(final DoubleSolenoid doubleSolenoid) {
        this(doubleSolenoid, false);
    }

    /**
     * Extends the piston outwards, if incorrect change whether or not the
     * {@linkplain DoublePistonController} needs to be inverted in it's constructor.
     */
    public void extend() {
        if (inverted) {
            doubleSolenoid.set(Value.kReverse);
        } else {
            doubleSolenoid.set(Value.kForward);
        }
    }

    /**
     * Retracts the piston inwards, if incorrect change whether or not the
     * {@linkplain DoublePistonController} needs to be inverted in it's constructor.
     */
    public void retract() {
        if (inverted) {
            doubleSolenoid.set(Value.kForward);
        } else {
            doubleSolenoid.set(Value.kReverse);
        }
    }

    /**
     * Retracts or extends the piston depending on its current state.
     */
    public void toggle() {
        if (doubleSolenoid.get() == Value.kReverse) {
            doubleSolenoid.set(Value.kForward);
        } else if (doubleSolenoid.get() == Value.kForward) {
            doubleSolenoid.set(Value.kReverse);
        }
    }

    public boolean get() {
        if (doubleSolenoid.get() == Value.kForward) {
            return true;
        }
        return false;
    }

    /**
     * Sets the piston to off, where neither a extending or a retracting force is
     * applied to the piston.
     */
    public void off() {
        doubleSolenoid.set(Value.kOff);
    }

    /**
     * Extends the piston outwards, if incorrect change whether or not the
     * {@linkplain DoublePistonController} needs to be inverted in it's constructor.
     * 
     * @param duration The Duration that the {@linkplain DoublePistonController}
     *                 should extend the piston for before retracting.
     */
    public void extendForTime(final double duration) {
        try {
            executorLock.lock();
            if (executorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + duration;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.getInstance().isDisabled()) {
                                break;
                            }
                            extend();
                            Timer.delay(.02);
                        }
                        retract();
                    } finally {
                        executorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Retarcts the piston inwards, if incorrect change whether or not the
     * {@linkplain DoublePistonController} needs to be inverted in it's constructor.
     * 
     * @param duration The Duration that the {@linkplain DoublePistonController}
     *                 should retract the piston for before extending.
     */
    public void retractForTime(final double duration) {
        try {
            executorLock.lock();
            if (executorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + duration;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.getInstance().isDisabled()) {
                                break;
                            }
                            retract();
                            extend();
                            Timer.delay(.02);
                        }
                        extend();
                    } finally {
                        executorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Retracts the piston.
     */
    public void disable() {
        retract();
    }

}