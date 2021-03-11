package frc.robot;

// FRC Imports
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

    /**
     * Joystick Deazone
     * 
     * @param JOYSTICK_DEADZONE The percent of error allowed, from -1.0 to 1.0, of
     *                          the {@linkplain Joystick} X and Y values.
     */
    public static final double JOYSTICK_DEADZONE = .08;

    /**
     * Max Drive Speed
     * 
     * @param MAX_Y_DRIVE_SPEED The maximum drive speed of the robot, from 0 to 1.0.
     */
    public static final double MAX_Y_DRIVE_SPEED = .8;
    public static final double MAX_X_DRIVE_SPEED = .25;

    /**
     * Drive Controller TalonFX CAN IDs
     * 
     * @param leftDriveCANIDArray  An int array representing the CAN IDs used for
     *                             the first left drive motors.
     * @param rightDriveCANIDArray An int array representing the CAN IDs used for
     *                             the first right drive motors.
     */
    public static final int[] leftDriveCANIDArray = new int[] { 1, 2 };
    public static final int[] rightDriveCANIDArray = new int[] { 3, 4 };

    /**
     * Gyro PID Values
     * 
     * @param gyro_kP The proportional value for the Gyro PID Controller.
     * @param gyro_kI The integral value for the Gyro PID Controller.
     * @param gyro_kD The derivative value for the Gyro PID Controller.
     */
    public static final double gyro_kP = .0045, gyro_kI = .0001, gyro_kD = .0001;

    /**
     * Limelight PID Values
     * 
     * @param limelight_kP The proportional value for the Limelight PID Controller.
     * @param limelight_kI The integral value for the Limelight PID Controller.
     * @param limelight_kD The derivative value for the Limelight PID Controller.
     */
    public static final double limelight_kP = .025, limelight_kI = .003, limelight_kD = .00175;

    /**
     * PCM CAN ID
     * 
     * @param pcmCANID An int representing the CAN ID of the PCM used for both the
     *                 {@linkplain Compressor} motor and all of the
     *                 {@linkplain Solenoid} and {@linkplain DoubleSolenoid}.
     */
    public static final int pcmCANID = 14;

    /**
     * Shooter Controller CAN IDs
     * 
     * @param shooterBrakeForwardChannel
     * @param shooterBrakeReverseChannel
     * @param lowerShooterCANID          An int representing the CAN ID of the
     *                                   {@linkplain TalonFX} motor.
     * @param upperShooterCANID          An int representing the CAN ID of the
     *                                   {@linkplain TalonFX} motor.
     * @param pickupCANID                An int representing the CAN ID of the
     *                                   {@linkplain TalonFX} motor.
     * @param feederCANIDArray           An int array containing the CAN IDs of the
     *                                   {@linkplain TalonSRX} motors.
     * @param targetLightChannel         The DIO channel to use for the
     *                                   {@linkplain DigitalOutput} to control the
     *                                   targeting light relay.
     *                                   <P>
     *                                   0-9 are on-board, 10-25 are on the MXP.
     */
    public static final int shooterBrakeForwardChannel = 0;
    public static final int shooterBrakeReverseChannel = 1;
    public static final int lowerShooterCANID = 5;
    public static final int upperShooterCANID = 6;
    public static final int pickupCANID = 7;
    public static final int[] feederCANIDArray = new int[] { 8, 9 };
    public static final int targetLightChannel = 0;

    /**
     * Shooter Controller Inversion
     * 
     * @param shooterInverted Whether or not the shooter motors need to be inverted.
     * @param pickupInverted  Whether or not the pickup motor needs to be inverted.
     * @param feederInverted  Wether or not the feeder motor needs to be inverted.
     */
    public static final boolean shooterInverted = true;
    public static final boolean pickupInverted = true;
    public static final boolean feederInverted = true;
    public static final boolean shooterBrakeInverted = true;

    /**
     * Shooter Motors PID Controller Settings for Velocity based PID Values.
     * <p>
     * PID Values may have to be adjusted based on the responsiveness of control
     * loop. kF: 1.0 represents output value to TalonFX at 100%, 6380/600 represents
     * Velocity units per 100 ms at 100% output.
     * 
     * @param shooter_kPIDLoopIndex Sets whether or not the integrated PID
     *                              Controller of the Talon is closed loop (0) or
     *                              open loop (1).
     * @param shooter_kTimeoutMs    The timeout in ms to give the PID Controller of
     *                              the Talon.
     * @param shooter_kP            The proportional value for the Shooter PID
     *                              Controller.
     * @param shooter_kI            The integral value for the Shooter PID
     *                              Controller.
     * @param shooter_kD            The derivative value for the Shooter PID
     *                              Controller.
     * @param shooter_kF            The feed-forward value for the Shooter PID
     *                              Controller.
     */
    public static final int shooter_kPIDLoopIndex = 0, shooter_kTimeoutMs = 20;
    public static final double shooter_kP = .01, shooter_kI = .0002, shooter_kD = .0001, shooter_kF = 0;
    // shooter_kF = 1.0 / (6380 / 600);

}