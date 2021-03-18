package frc.team3171.controllers;

//Rev Robotisc Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//Team 3171 Imports
import frc.robot.RobotProperties;

/**
 * @author Evan Lockwood
 */
public class ShooterShield implements RobotProperties{

    //Shield Motor
    private final CANSparkMax shieldMotor;


    public ShooterShield(){
        //Init the motors
        shieldMotor = new CANSparkMax(shieldCANID, MotorType.kBrushless);
        shieldMotor.restoreFactoryDefaults();
        shieldMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Sets the speed of the shield motor to the given value.
     * 
     * @param speed The speed, from -1.0 to 1.0, to set the wheel motor to.
     */

    public void setShieldSpeed(final double speed){
        shieldMotor.set(speed);
    }

    /**
     * Stops the shield from spinning
     */

    public void stopWheel(){
        shieldMotor.set(0);
    }

    /**
     * Disables all motors in the {@linkplain ShooterShield} class.
     */

    public void disable(){
        shieldMotor.disable();
    }

}