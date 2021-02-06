package frc.team3171.controllers;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.drive.TalonSRXMotorGroup;
import frc.team3171.pnuematics.DoublePistonController;

/**
 * @author Mark Ebert
 */
public class Climber implements RobotProperties {

    // Winch Motors
    private final TalonSRXMotorGroup winchMotors;

    // Trolly Motors
    private final TalonSRX trollyMotors;

    // Pneumatic Piston
    private final DoublePistonController climberArms;

    /**
     * Construtor
     * 
     * @throws Exception Throws a new exception if there are an invalid amount of
     *                   motors in the canIDArray.
     */
    public Climber() throws Exception {
        // Init the motors
        winchMotors = new TalonSRXMotorGroup(winchInverted, winchCANIDArray);
        trollyMotors = new TalonSRX(trollyCANID);
        trollyMotors.setInverted(trollyInverted);

        // Init the pneumatics
        climberArms = new DoublePistonController(pcmCANID, liftSolenoidForwardChannel, liftSolenoidReverseChannel,
                liftInverted);
    }

    /**
     * Sets the speed of the winch motors to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the all of the winch motors
     *              to.
     */
    public void setWinchSpeed(final double speed) {
        winchMotors.set(speed);
    }

    /**
     * Sets the speed of the Trolly motors to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the all of the Trolly motors
     *              to.
     */
    public void setTrollySpeed(final double speed) {
        trollyMotors.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Extends the climber arms.
     */
    public void extendClimber() {
        climberArms.extend();
    }

    /**
     * Retracts the climber arms.
     */
    public void retractClimber() {
        climberArms.retract();
    }

    /**
     * Toggles the climber arms.
     */
    public void toggleClimber() {
        climberArms.toggle();
    }

    public boolean isExtended() {
        return climberArms.get();
    }

    /**
     * Disables all motors in the {@link Climber} class.
     */
    public final void disable() {
        winchMotors.disable();
        trollyMotors.set(ControlMode.Disabled, 0);
        climberArms.disable();
    }

}