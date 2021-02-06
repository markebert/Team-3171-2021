package frc.team3171.drive;

// FRC Imports
import edu.wpi.first.wpilibj.SpeedController;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;;

/**
 * Encapsulates the {@link TalonFX} to comply with the {@link SpeedController}
 * to interface so that it can function within FRC designed drive trains.
 * 
 * @author Mark Ebert
 */
public class FRCTalonFX extends TalonFX implements SpeedController {

    /**
     * Constructor
     * 
     * @param deviceNumber [0,62]
     */
    public FRCTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void pidWrite(double output) {
        set(ControlMode.PercentOutput, output);
    }

    @Override
    public void set(double speed) {
        set(ControlMode.PercentOutput, speed);
    }

    @Override
    public double get() {
        return getMotorOutputPercent();
    }

    @Override
    public void disable() {
        set(ControlMode.Disabled, 0);
    }

    @Override
    public void stopMotor() {
        set(ControlMode.Disabled, 0);
    }

}