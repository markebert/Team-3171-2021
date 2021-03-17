package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

// FRC Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.drive.TalonSRXMotorGroup;
import frc.team3171.pnuematics.DoublePistonController;

/**
 * @author Mark Ebert
 */
public class Shooter implements RobotProperties {

    // Motor Controllers
    private final TalonFX lowerShooterMotor, upperShooterMotor, pickupMotor;
    private final TalonSRXMotorGroup feederMotors;

    // Relay for the targeting light
    private final Relay targetLightRelay;

    // Double Solenoid used to stop the balls in the feeder
    private final DoublePistonController shooterBrake;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean feederExecutorActive;

    /**
     * Constructor
     * 
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors in the feederCANIDArray.
     */
    public Shooter() throws Exception {
        // Init all of the motors
        lowerShooterMotor = new TalonFX(lowerShooterCANID);
        upperShooterMotor = new TalonFX(upperShooterCANID);
        pickupMotor = new TalonFX(pickupCANID);
        feederMotors = new TalonSRXMotorGroup(feederInverted, feederCANIDArray);
        targetLightRelay = new Relay(targetLightChannel, Direction.kForward);

        // Factory Default all motors to prevent unexpected behaviour
        lowerShooterMotor.configFactoryDefault();
        upperShooterMotor.configFactoryDefault();
        pickupMotor.configFactoryDefault();

        // Init the shooter motors and pid controller
        initShooterMotorsPID();

        // Inverts the pickup motor if needed
        pickupMotor.setInverted(pickupInverted);

        // Init the shooter brake
        shooterBrake = new DoublePistonController(pcmCANID, shooterBrakeForwardChannel, shooterBrakeReverseChannel,
                shooterBrakeInverted);

        // Initialize the executor service for concurrency
        executorService = Executors.newSingleThreadExecutor();

        executorLock = new ReentrantLock(true);

        // Initialize the AtomicBooleans to control the thread executors
        feederExecutorActive = new AtomicBoolean(false);
    }

    /**
     * Sets up the shooter motors and their PID controllers.
     * 
     * @param shooterInverted Whether or not the shooter motors need to be inverted.
     */
    private final void initShooterMotorsPID() {
        // Set if any motors need to be inverted
        lowerShooterMotor.setInverted(!shooterInverted);
        upperShooterMotor.setInverted(!shooterInverted);

        // Config sensor used for Shooter Motor Velocity PID Controller
        lowerShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, shooter_kPIDLoopIndex,
                shooter_kTimeoutMs);
        upperShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, shooter_kPIDLoopIndex,
                shooter_kTimeoutMs);

        /**
         * Set the sensor phase accordingly. Positive sensor reading should match Green
         * (blinking) Leds on Talon when the motor is being driven.
         */
        lowerShooterMotor.setSensorPhase(shooterInverted);
        upperShooterMotor.setSensorPhase(!shooterInverted);

        // Resets the ingrated encoders on the Shooter Motors
        lowerShooterMotor.setSelectedSensorPosition(0);
        upperShooterMotor.setSelectedSensorPosition(0);

        // Config the peak and nominal outputs of the Shooter Motors for PID Control
        lowerShooterMotor.configNominalOutputForward(0, shooter_kTimeoutMs);
        lowerShooterMotor.configNominalOutputReverse(0, shooter_kTimeoutMs);
        lowerShooterMotor.configPeakOutputForward(1, shooter_kTimeoutMs);
        lowerShooterMotor.configPeakOutputReverse(-1, shooter_kTimeoutMs);

        upperShooterMotor.configNominalOutputForward(0, shooter_kTimeoutMs);
        upperShooterMotor.configNominalOutputReverse(0, shooter_kTimeoutMs);
        upperShooterMotor.configPeakOutputForward(1, shooter_kTimeoutMs);
        upperShooterMotor.configPeakOutputReverse(-1, shooter_kTimeoutMs);

        // Config the Velocity closed loop values in slot0
        lowerShooterMotor.config_kP(shooter_kPIDLoopIndex, shooter_kP, shooter_kTimeoutMs);
        lowerShooterMotor.config_kI(shooter_kPIDLoopIndex, shooter_kI, shooter_kTimeoutMs);
        lowerShooterMotor.config_kD(shooter_kPIDLoopIndex, shooter_kD, shooter_kTimeoutMs);
        lowerShooterMotor.config_kF(shooter_kPIDLoopIndex, shooter_kF, shooter_kTimeoutMs);

        upperShooterMotor.config_kP(shooter_kPIDLoopIndex, shooter_kP, shooter_kTimeoutMs);
        upperShooterMotor.config_kI(shooter_kPIDLoopIndex, shooter_kI, shooter_kTimeoutMs);
        upperShooterMotor.config_kD(shooter_kPIDLoopIndex, shooter_kD, shooter_kTimeoutMs);
        upperShooterMotor.config_kF(shooter_kPIDLoopIndex, shooter_kF, shooter_kTimeoutMs);
    }

    /**
     * Controls the relay to turn on or off the targeting light on the shooter.
     * 
     * @param enable True to enable the targeting light, false to disable it.
     */
    public void enableTargetingLight(final boolean enable) {
        if (enable) {
            targetLightRelay.set(Value.kOn);
        } else {
            targetLightRelay.set(Value.kOff);
        }
    }

    /**
     * Controls the {@linkplain DoubleSolenoid} to set whether or not the shooter
     * brake should be engaged or disengaged.
     * 
     * @param enable True to engaged the brake to prevent ball movement, false to
     *               disengaged it and allow the balls to pass.
     */
    public void setShooterBrake(final boolean enable) {
        if (enable) {
            shooterBrake.extend();
        } else {
            shooterBrake.retract();
        }
    }

    /**
     * Disengages the shooter brake to allow the balls to pass.
     */
    public void disengageShooterBrake() {
        setShooterBrake(false);
    }

    /**
     * Engages the shooter brake to prevent the balls from passing.
     */
    public void engageShooterBrake() {
        setShooterBrake(true);
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     * 
     * @param lowerShooterSpeed The speed, from -1.0 to 1.0, to set the lower
     *                          shooter motor to.
     * @param upperShooterSpeed The speed, from -1.0 to 1.0, to set the upper
     *                          shooter motor to.
     */
    public void setShooterSpeed(final double lowerShooterSpeed, final double upperShooterSpeed) {
        lowerShooterMotor.set(ControlMode.PercentOutput, lowerShooterSpeed);
        upperShooterMotor.set(ControlMode.PercentOutput, upperShooterSpeed);
        if (lowerShooterSpeed != 0 || upperShooterSpeed != 0) {
            enableTargetingLight(true);
        } else {
            enableTargetingLight(false);
        }
    }

    /**
     * Sets the speed of the shooter motors to the given value.
     * 
     * @param shooterSpeed The speed, from -1.0 to 1.0, to set the all of the
     *                     shooter motors to.
     */
    public void setShooterSpeed(final double shooterSpeed) {
        setShooterSpeed(shooterSpeed, shooterSpeed);
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     * 
     * @param lowerShooterRPM The RPM to set the lower shooter motor to.
     * @param upperShooterRPM The RPM to set the upper shooter motor to.
     */
    public void setShooterVelocity(final int lowerShooterRPM, final int upperShooterRPM) {
        /**
         * First check if either desired RPM is 0, if so lets the electronic brake
         * handle the slow done rather then the PID Controller.
         * <p>
         * Otherwise, converts the desired shooterRPM to units / 100ms.
         * <p>
         * (2048-Units/Rev * shooterRPM) / 600-100ms/min in either direction.
         * <p>
         * Velocity setpoint is in units/100ms.
         */
        if (lowerShooterRPM == 0) {
            lowerShooterMotor.set(ControlMode.PercentOutput, 0);
            enableTargetingLight(false);
        } else {
            final double lowerTargetVelocity_UnitsPer100ms = (lowerShooterRPM * 2048.0) / 600;
            lowerShooterMotor.set(ControlMode.Velocity, lowerTargetVelocity_UnitsPer100ms);
            enableTargetingLight(true);
        }
        if (upperShooterRPM == 0) {
            upperShooterMotor.set(ControlMode.PercentOutput, 0);
            enableTargetingLight(false);
        } else {
            final double upperTargetVelocity_UnitsPer100ms = (upperShooterRPM * 2048.0) / 600;
            upperShooterMotor.set(ControlMode.Velocity, upperTargetVelocity_UnitsPer100ms);
            enableTargetingLight(true);
        }
    }

    /**
     * Sets the RPM of the shooter motors to the given value.
     * 
     * @param shooterRPM The RPM to set the all of the shooter motors to.
     */
    public void setShooterVelocity(final int shooterRPM) {
        setShooterVelocity(shooterRPM, shooterRPM);
    }

    /**
     * Returns the velocity of the lower shooter motor in RPM, converted from Units
     * per 100ms.
     * 
     * @return The RPM of the lower shooter motor.
     */
    public double getLowerShooterVelocity() {
        return (lowerShooterMotor.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    /**
     * Returns the velocity of the upper shooter motor in RPM, converted from Units
     * per 100ms.
     * 
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterVelocity() {
        return (upperShooterMotor.getSelectedSensorVelocity() / 2048.0) * 600;
    }


    /**
     * Returns the target velocity of the lower shooter motor in RPM, converted from
     * Units per 100ms.
     * 
     * @return The target RPM of the lower shooter motor.
     */
    public double getLowerShooterTargetVelocity() {
        if (lowerShooterMotor.getControlMode() == ControlMode.Velocity) {
            return (lowerShooterMotor.getClosedLoopTarget() / 2048.0) * 600;
        }
        return 0;
    }

    /**
     * Returns the target velocity of the upper shooter motor in RPM, converted from
     * Units per 100ms.
     * 
     * @return The RPM of the upper shooter motor.
     */
    public double getUpperShooterTargetVelocity() {
        if (upperShooterMotor.getControlMode() == ControlMode.Velocity) {
            return (upperShooterMotor.getClosedLoopTarget() / 2048.0) * 600;
        }
        return 0;
    }

    /**
     * Returns the speed of the lower shooter motor in percent.
     * 
     * @return The speed of the lower shooter motor, from -1.0 to 1.0.
     */
    public double getLowerShooterSpeed() {
        return lowerShooterMotor.getMotorOutputPercent();
    }

    /**
     * Returns the speed of the upper shooter motor.
     * 
     * @return The speed of the upper shooter motor in percent, from -1.0 to 1.0.
     */
    public double getUpperShooterSpeed() {
        return upperShooterMotor.getMotorOutputPercent();
    }

    /**
     * Sets the speed of the feeder motors to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the feeder motors to.
     */
    public void setFeederSpeed(final double speed) {
        feederMotors.set(speed);
    }

    /**
     * Sets the speed of the feeder motors to the given value and keeps them running
     * for the desired time.
     * 
     * @param speed   The speed, from -1.0 to 1.0, to set the feeder motor to.
     * @param runTime The amount of time, in seconds, to keep the motors spinning
     *                for.
     */
    public void runFeeder(final double speed, final double runTime) {
        try {
            executorLock.lock();
            if (feederExecutorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + runTime;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.getInstance().isDisabled()) {
                                break;
                            }
                            setFeederSpeed(speed);
                            Timer.delay(.02);
                        }
                        setFeederSpeed(0);
                    } finally {
                        feederExecutorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the speed of the pickup motor to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the pickup motors to.
     */
    public void setPickupSpeed(final double speed) {
        pickupMotor.set(ControlMode.PercentOutput, speed);

    }

    /**
     * Disables all motors in the {@link Shooter} class.
     */
    public final void disable() {
        lowerShooterMotor.set(ControlMode.Disabled, 0);
        upperShooterMotor.set(ControlMode.Disabled, 0);
        pickupMotor.set(ControlMode.Disabled, 0);
        feederMotors.disable();
        targetLightRelay.set(Value.kOff);
        shooterBrake.disable();
    }

}